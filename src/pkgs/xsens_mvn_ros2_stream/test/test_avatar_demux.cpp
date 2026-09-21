// Copyright (c) 2026, Xsens Technologies B.V.
// SPDX-License-Identifier: BSD-3-Clause
//
// Multi-avatar demultiplexing, driven by real MVN datagrams.
//
// MVN sends every avatar to the same UDP port, tagged only by an avatar id in
// the header, so one avatar's datagram can silently overwrite another's pose.
// These tests replay a recorded three-avatar scene (two suits plus a tracked
// object) into a real socket and assert the avatars stay separated.
//
// The fixture is a trimmed capture of an actual MVN stream rather than
// synthesised bytes, so it also guards against the wire format drifting.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <xsens_mvn_ros2_stream/xsens_stream_client.hpp>

namespace
{

constexpr int kTestPortBase = 19763;
constexpr uint8_t kAvatarIdOffset = MVN_AVATAR_ID_OFFSET;

/// Ports are per-test so a lingering socket from one cannot disturb the next.
int nextPort()
{
  static std::atomic<int> counter{0};
  return kTestPortBase + counter++;
}

std::string fixturePath()
{
  return std::string(TEST_DATA_DIR) + "/mvn_three_avatars.bin";
}

/// Reads the length-prefixed datagram records written by the capture tool.
std::vector<std::vector<char>> loadFixture()
{
  std::vector<std::vector<char>> out;
  std::ifstream in(fixturePath(), std::ios::binary);
  EXPECT_TRUE(in.good()) << "Cannot open fixture " << fixturePath();
  while (in.good()) {
    unsigned char len_be[4];
    in.read(reinterpret_cast<char *>(len_be), 4);
    if (in.gcount() != 4) {
      break;
    }
    const uint32_t len = (static_cast<uint32_t>(len_be[0]) << 24) |
      (static_cast<uint32_t>(len_be[1]) << 16) |
      (static_cast<uint32_t>(len_be[2]) << 8) | len_be[3];
    std::vector<char> d(len);
    in.read(d.data(), len);
    if (in.gcount() != static_cast<std::streamsize>(len)) {
      break;
    }
    out.push_back(std::move(d));
  }
  return out;
}

/// Replays datagrams to 127.0.0.1:<port> in a loop until stopped, the way MVN
/// would keep streaming.
class Replayer
{
public:
  Replayer(int port, std::vector<std::vector<char>> datagrams)
  : m_port(port), m_datagrams(std::move(datagrams)) {}

  ~Replayer() {stop();}

  void start()
  {
    m_running = true;
    m_thread = std::thread(
      [this]() {
        int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(m_port));
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        while (m_running) {
          for (const auto & d : m_datagrams) {
            if (!m_running) {
              break;
            }
            ::sendto(
              sock, d.data(), d.size(), 0,
              reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          }
        }
        ::close(sock);
      });
  }

  void stop()
  {
    m_running = false;
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

private:
  int m_port;
  std::vector<std::vector<char>> m_datagrams;
  std::atomic<bool> m_running{false};
  std::thread m_thread;
};

/// Re-tags a datagram as belonging to another avatar, which is all MVN itself
/// varies between avatars of the same kind.
std::vector<char> withAvatarId(std::vector<char> d, uint8_t avatar)
{
  if (d.size() > kAvatarIdOffset) {
    d[kAvatarIdOffset] = static_cast<char>(avatar);
  }
  return d;
}

uint8_t avatarIdOf(const std::vector<char> & d)
{
  return static_cast<uint8_t>(d[kAvatarIdOffset]);
}

int messageTypeOf(const std::vector<char> & d)
{
  return std::stoi(std::string(d.data() + 4, 2), nullptr, 16);
}

/// Polls until `pred` holds or the timeout expires; keeps the tests from
/// depending on how quickly the acquisition thread gets scheduled.
template<typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds timeout = std::chrono::seconds(10))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return pred();
}

rclcpp::Logger testLogger()
{
  return rclcpp::get_logger("test_avatar_demux");
}

}  // namespace

TEST(AvatarDemux, FixtureHoldsThreeAvatars)
{
  // Guards the fixture itself: if it is re-recorded, the scene it captures
  // must still be two body avatars plus one object.
  const auto datagrams = loadFixture();
  ASSERT_FALSE(datagrams.empty());

  std::map<uint8_t, int> pose_segments;
  for (const auto & d : datagrams) {
    if (messageTypeOf(d) == 0x02) {          // PoseQuaternion
      pose_segments[avatarIdOf(d)] = static_cast<unsigned char>(d[11]);
    }
  }
  ASSERT_EQ(pose_segments.size(), 3u);
  EXPECT_EQ(pose_segments[0], 63);
  EXPECT_EQ(pose_segments[1], 63);
  EXPECT_EQ(pose_segments[2], 1);
}

TEST(AvatarDemux, TracksEveryAvatarSeparately)
{
  const int port = nextPort();
  Replayer replayer(port, loadFixture());
  replayer.start();

  XsensStreamClient client(testLogger(), port, /*avatar_id=*/0, /*track_all_avatars=*/true);
  ASSERT_TRUE(client.init());

  ASSERT_TRUE(waitFor([&]() {return client.avatarIds().size() == 3;}))
    << "expected three avatars, saw " << client.avatarIds().size();

  // Each avatar keeps its own model: the object's single segment must not
  // truncate the suits, and the suits must not inflate the object.
  EXPECT_EQ(client.getAvatarSegments(0).size(), 63u);
  EXPECT_EQ(client.getAvatarSegments(1).size(), 63u);
  EXPECT_EQ(client.getAvatarSegments(2).size(), 1u);

  EXPECT_FALSE(client.isObject(0));
  EXPECT_FALSE(client.isObject(1));
  EXPECT_TRUE(client.isObject(2));
}

TEST(AvatarDemux, ObjectHasNoJointsButBodiesDo)
{
  const int port = nextPort();
  Replayer replayer(port, loadFixture());
  replayer.start();

  XsensStreamClient client(testLogger(), port, 0, true);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.avatarIds().size() == 3;}));

  EXPECT_FALSE(client.getAvatarJoints(0).empty());
  EXPECT_TRUE(client.getAvatarJoints(2).empty())
    << "a rigid object should not have joints";

  // The object's only segment is the rigid-body root.
  const auto object_segments = client.getAvatarSegments(2);
  ASSERT_EQ(object_segments.size(), 1u);
  EXPECT_EQ(object_segments.begin()->first, "base_link");
}

TEST(AvatarDemux, SingleAvatarModeIgnoresTheOthers)
{
  const int port = nextPort();
  Replayer replayer(port, loadFixture());
  replayer.start();

  // The default: only avatar 0 is parsed, so the object's one-segment
  // datagrams can never overwrite the suit's pose.
  XsensStreamClient client(testLogger(), port, /*avatar_id=*/0, /*track_all_avatars=*/false);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return !client.getSegments().empty();}));

  // Give the other avatars ample opportunity to leak in.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto ids = client.avatarIds();
  ASSERT_EQ(ids.size(), 1u);
  EXPECT_EQ(ids.front(), 0);
  EXPECT_EQ(client.getSegments().size(), 63u);
}

TEST(AvatarDemux, PrimaryCanBeAnObject)
{
  const int port = nextPort();
  Replayer replayer(port, loadFixture());
  replayer.start();

  // An object never sends joint angles, so model building must not block
  // waiting for them.
  XsensStreamClient client(testLogger(), port, /*avatar_id=*/2, /*track_all_avatars=*/false);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return !client.getSegments().empty();}));

  EXPECT_EQ(client.getSegments().size(), 1u);
  EXPECT_TRUE(client.isObject(2));
  EXPECT_TRUE(client.getJoints().empty());
}

TEST(AvatarDemux, RebuildsModelWhenSegmentCountChanges)
{
  const auto fixture = loadFixture();

  // Phase 1: avatar 3 is the object's one-segment stream.
  std::vector<std::vector<char>> object_only;
  for (const auto & d : fixture) {
    if (avatarIdOf(d) == 2) {
      object_only.push_back(withAvatarId(d, 3));
    }
  }
  ASSERT_FALSE(object_only.empty());

  const int port = nextPort();
  Replayer object_phase(port, object_only);
  object_phase.start();

  XsensStreamClient client(testLogger(), port, /*avatar_id=*/3, /*track_all_avatars=*/true);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.isObject(3);}));
  EXPECT_EQ(client.getAvatarSegments(3).size(), 1u);

  object_phase.stop();

  // Phase 2: the same avatar id now carries a full body, as happens when MVN
  // recomposes a scene and renumbers its avatars.
  std::vector<std::vector<char>> body_now;
  for (const auto & d : fixture) {
    if (avatarIdOf(d) == 0) {
      body_now.push_back(withAvatarId(d, 3));
    }
  }
  ASSERT_FALSE(body_now.empty());

  Replayer body_phase(port, body_now);
  body_phase.start();

  EXPECT_TRUE(waitFor([&]() {return client.getAvatarSegments(3).size() == 63u;}))
    << "model was not rebuilt; still "
    << client.getAvatarSegments(3).size() << " segment(s)";
  EXPECT_FALSE(client.isObject(3));
}

TEST(AvatarDemux, QuietAvatarStopsAdvancingItsTimestamp)
{
  const auto fixture = loadFixture();

  const int port = nextPort();
  Replayer all_avatars(port, fixture);
  all_avatars.start();

  XsensStreamClient client(testLogger(), port, 0, /*track_all_avatars=*/true);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.avatarIds().size() == 3;}));

  all_avatars.stop();

  // The object drops out of the scene while the suits keep streaming, which is
  // what avatar_stale_timeout keys off to drop an avatar from TF.
  std::vector<std::vector<char>> without_object;
  for (const auto & d : fixture) {
    if (avatarIdOf(d) != 2) {
      without_object.push_back(d);
    }
  }
  Replayer suits_only(port, without_object);
  suits_only.start();

  // Let any datagrams still in flight drain before sampling.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const int64_t suit_before = client.avatarLastDataTimeNs(0);
  const int64_t object_before = client.avatarLastDataTimeNs(2);
  ASSERT_GT(object_before, 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  EXPECT_GT(client.avatarLastDataTimeNs(0), suit_before)
    << "a streaming avatar should keep advancing";
  EXPECT_EQ(client.avatarLastDataTimeNs(2), object_before)
    << "an avatar that stopped sending should not appear fresh";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
