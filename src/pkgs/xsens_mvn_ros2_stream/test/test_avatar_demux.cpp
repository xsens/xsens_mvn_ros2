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
#include <xsens_mvn_sdk/quaterniondatagram.h>  // cpplint files a .h header here

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
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

// ---- Header layout of an MVN datagram (see xsens_mvn_sdk/datagram.cpp) ----
constexpr size_t kHeaderSize = 24;
constexpr size_t kItemCountOffset = 11;
constexpr size_t kBodyCountOffset = 17;
constexpr size_t kPropCountOffset = 18;
constexpr size_t kFingerCountOffset = 19;

/// Per-item payload size of the three per-segment datagram types, or 0 for
/// types that are not laid out per segment.
size_t itemSizeOf(const std::vector<char> & d)
{
  switch (messageTypeOf(d)) {
    case 0x02: return 32;   // PoseQuaternion: id, pos[3], quat[4]
    case 0x21: return 40;   // LinearSegmentKinematics: id, pos[3], vel[3], acc[3]
    case 0x22: return 44;   // AngularSegmentKinematics: id, quat[4], vel[3], acc[3]
    default: return 0;
  }
}

void putInt32(std::vector<char> & d, size_t at, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v);
  d[at] = static_cast<char>(u >> 24);
  d[at + 1] = static_cast<char>(u >> 16);
  d[at + 2] = static_cast<char>(u >> 8);
  d[at + 3] = static_cast<char>(u);
}

int32_t getInt32(const std::vector<char> & d, size_t at)
{
  return static_cast<int32_t>(
    (static_cast<uint32_t>(static_cast<unsigned char>(d[at])) << 24) |
    (static_cast<uint32_t>(static_cast<unsigned char>(d[at + 1])) << 16) |
    (static_cast<uint32_t>(static_cast<unsigned char>(d[at + 2])) << 8) |
    static_cast<uint32_t>(static_cast<unsigned char>(d[at + 3])));
}

void putFloat(std::vector<char> & d, size_t at, float v)
{
  uint32_t u;
  std::memcpy(&u, &v, sizeof u);
  putInt32(d, at, static_cast<int32_t>(u));
}

/// Position written into every injected prop segment, so a test can tell a
/// prop's data apart from the body's.
constexpr float kPropPosition[3] = {1.5f, -2.5f, 0.75f};

/// Rewrites a per-segment datagram the way MVN lays it out when the actor
/// holds @p props props: the new segments go right after the 23 body segments,
/// the finger segments that follow are renumbered, and the header's item and
/// prop counts are updated.  Datagram types without per-segment items are
/// returned untouched.
std::vector<char> withProps(std::vector<char> d, int props)
{
  const size_t item_size = itemSizeOf(d);
  if (item_size == 0) {
    return d;
  }
  constexpr int kBody = 23;
  const int items = static_cast<unsigned char>(d[kItemCountOffset]);
  if (items < kBody) {
    return d;  // an object: props are not part of its stream
  }
  // Renumber everything after the body first, while the offsets still hold.
  for (int i = kBody; i < items; ++i) {
    const size_t at = kHeaderSize + i * item_size;
    putInt32(d, at, getInt32(d, at) + props);
  }
  std::vector<char> prop_items(item_size * props, 0);
  for (int p = 0; p < props; ++p) {
    putInt32(prop_items, p * item_size, kBody + 1 + p);
    if (messageTypeOf(d) == 0x02) {
      for (int k = 0; k < 3; ++k) {
        putFloat(prop_items, p * item_size + 4 + 4 * k, kPropPosition[k]);
      }
      // A non-identity orientation: SkeletonPublisher treats identity as "no
      // data yet", and a real prop never reports exactly identity either.
      putFloat(prop_items, p * item_size + 16, 0.7071068f);
      putFloat(prop_items, p * item_size + 20, 0.7071068f);
    }
  }
  d.insert(
    d.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + kBody * item_size),
    prop_items.begin(), prop_items.end());
  d[kItemCountOffset] = static_cast<char>(items + props);
  d[kPropCountOffset] = static_cast<char>(props);
  return d;
}

/// Drops the finger segments from a per-segment body datagram, as streamed by
/// an actor without gloves.  Must be applied before withProps().
std::vector<char> withoutFingers(std::vector<char> d)
{
  const size_t item_size = itemSizeOf(d);
  if (item_size == 0) {
    return d;
  }
  constexpr int kBody = 23;
  const int items = static_cast<unsigned char>(d[kItemCountOffset]);
  if (items <= kBody) {
    return d;
  }
  d.resize(kHeaderSize + kBody * item_size);
  d[kItemCountOffset] = static_cast<char>(kBody);
  d[kFingerCountOffset] = 0;
  return d;
}

/// Avatar 0's datagrams, each passed through @p transform.
template<typename Transform>
std::vector<std::vector<char>> primaryAvatarWith(Transform transform)
{
  std::vector<std::vector<char>> out;
  for (const auto & d : loadFixture()) {
    if (avatarIdOf(d) == 0) {
      out.push_back(transform(d));
    }
  }
  return out;
}

std::vector<char> firstPoseDatagram(const std::vector<std::vector<char>> & datagrams)
{
  for (const auto & d : datagrams) {
    if (messageTypeOf(d) == 0x02) {
      return d;
    }
  }
  throw std::runtime_error("no pose datagram in fixture");
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

  // The pose header also spells out the segment layout, which the prop tests
  // below build on: 23 body + 0 prop + 40 finger segments for a suit.
  const auto pose = firstPoseDatagram(datagrams);
  EXPECT_EQ(static_cast<unsigned char>(pose[kBodyCountOffset]), 23);
  EXPECT_EQ(static_cast<unsigned char>(pose[kPropCountOffset]), 0);
  EXPECT_EQ(static_cast<unsigned char>(pose[kFingerCountOffset]), 40);
}

TEST(AvatarDemux, PoseHeaderCountsAreParsed)
{
  // The SDK used to discard these header bytes; the link builder depends on
  // them to tell a prop from a finger segment.
  const auto pose = withProps(firstPoseDatagram(loadFixture()), 2);
  QuaternionDatagram datagram;
  datagram.deserialize(pose.data());
  EXPECT_EQ(datagram.dataCount(), 65);
  EXPECT_EQ(datagram.bodySegmentCount(), 23);
  EXPECT_EQ(datagram.propCount(), 2);
  EXPECT_EQ(datagram.fingerSegmentCount(), 40);

  const auto items = datagram.getData();
  ASSERT_EQ(items.size(), 65u);
  EXPECT_EQ(items[23].segmentId, 24);   // first prop
  EXPECT_EQ(items[24].segmentId, 25);   // second prop
  EXPECT_EQ(items[25].segmentId, 26);   // left carpus, renumbered
  EXPECT_EQ(items.back().segmentId, 65);
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

TEST(AvatarDemux, KeepsWaitingWhenTheStreamStartsLate)
{
  // The socket has a 1 s receive timeout.  A stream that starts after that
  // (MVN not yet streaming when the node comes up, or paused for calibration)
  // must still be picked up within the 30 s model-build deadline, rather than
  // the first timeout being mistaken for an empty datagram and ending
  // acquisition for good.
  const int port = nextPort();
  XsensStreamClient client(testLogger(), port, 0, /*track_all_avatars=*/false);

  std::atomic<bool> init_result{false};
  std::thread init_thread([&]() {init_result = client.init();});

  // Sit through more than two receive timeouts before anything is sent.
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  EXPECT_FALSE(client.isActive()) << "nothing has been streamed yet";

  Replayer replayer(port, loadFixture());
  replayer.start();
  init_thread.join();

  EXPECT_TRUE(init_result) << "init() should succeed once the stream starts";
  EXPECT_TRUE(waitFor([&]() {return !client.getSegments().empty();}));
}

TEST(AvatarDemux, PropSegmentsSitBetweenBodyAndFingers)
{
  // An actor holding one prop, with gloves: 23 + 1 + 40 segments.  Before the
  // header counts were read, the prop was published as the left carpus, every
  // finger shifted by one and the last right-hand segment was dropped.
  const int port = nextPort();
  Replayer replayer(port, primaryAvatarWith([](const auto & d) {return withProps(d, 1);}));
  replayer.start();

  XsensStreamClient client(testLogger(), port, 0, false);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.getSegments().size() == 64u;}))
    << "expected 64 segments, saw " << client.getSegments().size();

  const auto segments = client.getSegments();
  ASSERT_TRUE(segments.count("prop_1"));
  ASSERT_TRUE(segments.count("left_carpus"));
  ASSERT_TRUE(segments.count("right_fifth_dp")) << "last right-hand segment must survive";
  EXPECT_FALSE(segments.count("prop_2"));
  EXPECT_FALSE(client.isObject(0));

  // The prop carries the data that was injected for it, not a finger's.
  const auto & prop = segments.at("prop_1");
  EXPECT_FLOAT_EQ(prop.position.x(), kPropPosition[0]);
  EXPECT_FLOAT_EQ(prop.position.y(), kPropPosition[1]);
  EXPECT_FLOAT_EQ(prop.position.z(), kPropPosition[2]);

  // The hands keep their own data: the left carpus (now segment 25) must be
  // the same segment that was number 24 before the prop was inserted.  The
  // fixture holds two frames, so the published value may come from either.
  std::vector<float> carpus_x;
  for (const auto & d : loadFixture()) {
    if (avatarIdOf(d) != 0 || messageTypeOf(d) != 0x02) {
      continue;
    }
    const size_t carpus_at = kHeaderSize + 23 * 32;
    ASSERT_EQ(getInt32(d, carpus_at), 24);
    float x;
    const uint32_t ux = static_cast<uint32_t>(getInt32(d, carpus_at + 4));
    std::memcpy(&x, &ux, sizeof x);
    carpus_x.push_back(x);
  }
  ASSERT_FALSE(carpus_x.empty());
  const float published_x = static_cast<float>(segments.at("left_carpus").position.x());
  EXPECT_NE(std::find(carpus_x.begin(), carpus_x.end(), published_x), carpus_x.end())
    << "left_carpus x = " << published_x << " matches neither frame's segment 24";

  // A stable layout must not keep rebuilding the model, which would also
  // starve the joint model.
  EXPECT_TRUE(waitFor([&]() {return !client.getJoints().empty();}));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(client.getSegments().size(), 64u);
  EXPECT_FALSE(client.getJoints().empty());
}

TEST(AvatarDemux, PropsWithoutGlovesAreNotMistakenForFingers)
{
  // Two props and no finger data: 25 segments.  Inferred from the item count
  // alone this looks like a one-segment hand on each side.
  const int port = nextPort();
  Replayer replayer(
    port, primaryAvatarWith([](const auto & d) {return withProps(withoutFingers(d), 2);}));
  replayer.start();

  XsensStreamClient client(testLogger(), port, 0, false);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.getSegments().size() == 25u;}))
    << "expected 25 segments, saw " << client.getSegments().size();

  const auto segments = client.getSegments();
  EXPECT_TRUE(segments.count("prop_1"));
  EXPECT_TRUE(segments.count("prop_2"));
  EXPECT_FALSE(segments.count("left_carpus"));
  EXPECT_FALSE(segments.count("right_carpus"));
  EXPECT_TRUE(segments.count("pelvis"));
  EXPECT_FALSE(client.isObject(0));
  EXPECT_TRUE(waitFor([&]() {return !client.getJoints().empty();}));
}

TEST(AvatarDemux, OnePropWithoutGlovesIsNotDropped)
{
  // 24 segments: an odd count beyond the body used to round to zero finger
  // segments, skip the prop, and then rebuild the model on every frame.
  const int port = nextPort();
  Replayer replayer(
    port, primaryAvatarWith([](const auto & d) {return withProps(withoutFingers(d), 1);}));
  replayer.start();

  XsensStreamClient client(testLogger(), port, 0, false);
  ASSERT_TRUE(client.init());
  ASSERT_TRUE(waitFor([&]() {return client.getSegments().size() == 24u;}))
    << "expected 24 segments, saw " << client.getSegments().size();
  EXPECT_TRUE(client.getSegments().count("prop_1"));
  EXPECT_TRUE(waitFor([&]() {return !client.getJoints().empty();}))
    << "joints never built: the model is probably being rebuilt every frame";
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(client.getJoints().empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
