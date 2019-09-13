#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensors_si2019/HILNetSimTracing.h>
#include <sensors_si2019/utils.hpp>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace sensors_si2019 {

using namespace dccomms_packets;
using namespace utils;

HILNetSimTracing::HILNetSimTracing() : NetSimTracing() {
  e1_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer1/velocity", 1);
  e2_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer2/velocity", 1);
  e3_pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer3/velocity", 1);

  leader1_pub =
      node.advertise<geometry_msgs::TwistStamped>("/uwsim/leader1/velocity", 1);
  e1_2pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer1_2/velocity", 1);
  e2_2pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer2_2/velocity", 1);
  e3_2pub = node.advertise<geometry_msgs::TwistStamped>(
      "/uwsim/explorer3_2/velocity", 1);
}

void HILNetSimTracing::PacketTransmitting(std::string path,
                                          ROSCommsDevicePtr dev,
                                          ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  //  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
  //       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
  //       header.GetPacketSize());
}

void HILNetSimTracing::PacketDropsUpdated(std::string path, uint32_t oldValue,
                                          uint32_t newValue) {
  // Info("[{}] PKTDROPS {}", path, newValue);
}

void HILNetSimTracing::TxFifoUpdated(std::string path, uint32_t oldValue,
                                     uint32_t newValue) {
  // Info("[{}] TXFIFO {}", path, newValue);
}

void HILNetSimTracing::MacPacketDropsUpdated(std::string path,
                                             uint32_t oldValue,
                                             uint32_t newValue) {
  // Info("[{}] MAC PKTDROPS {}", path, newValue);
}

void HILNetSimTracing::MacTxFifoUpdated(std::string path, uint32_t oldValue,
                                        uint32_t newValue) {
  // Info("[{}] MAC TXFIFO {}", path, newValue);
}

void HILNetSimTracing::PacketError(std::string path, ROSCommsDevicePtr dev,
                                   ns3ConstPacketPtr pkt, bool propErr,
                                   bool colErr) {

  NetsimHeader header;
  pkt->PeekHeader(header);
  //  if (propErr) {
  //    if (!colErr) {
  //      Warn("[{}] PERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
  //           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
  //           header.GetPacketSize());
  //    } else {

  //      Warn("[{}] MERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
  //           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
  //           header.GetPacketSize());
  //    }
  //  } else {

  //    Warn("[{}] COL -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
  //         dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
  //         header.GetPacketSize());
  //  }
}

void HILNetSimTracing::PacketReceived(std::string path, ROSCommsDevicePtr dev,
                                      ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  //  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
  //       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
  //       header.GetPacketSize());
}

void HILNetSimTracing::MacRx(std::string path, ROSCommsDevicePtr dev,
                             ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  //  Info("[{}] MAC RX -- ID: {} ; MAC: {} ; Size: {}", path,
  //  dev->GetDccommsId(),
  //       dev->GetMac(), header.GetSize());
}

void HILNetSimTracing::MacTx(std::string path, ROSCommsDevicePtr dev,
                             ns3ConstPacketPtr pkt) {
  AquaSimHeader header;
  pkt->PeekHeader(header);
  //  Info("[{}] MAC TX -- ID: {} ; MAC: {} ; Size: {}", path,
  //  dev->GetDccommsId(),
  //       dev->GetMac(), header.GetSize());
}

void HILNetSimTracing::ShowPosition(string path, ROSCommsDevicePtr dev,
                                    const tf::Vector3 &pos) {

  // Info("[{}] POS: {} {} {}", dev->GetMac(), pos.getX(), pos.getY(),
  // pos.getZ());
}

void HILNetSimTracing::Configure() {
  SetLogName("uwsim_netsim_scripts");

  // The logging is managed by a spdlog (https://github.com/gabime/spdlog)
  // wrapper (https://github.com/dcentelles/cpplogging).
  // By default, all log messages will be prefixed by the script time in seconds
  // trying nanoseconds resolution (using the spdlog::formatter
  // dccomms_ros::NetsimLogFormatter)
  // Uncomment and customize the code below for adding more fields
  // to the log message's prefix
  // (https://github.com/gabime/spdlog/wiki/3.-Custom-formattingges):
  //  SetLogFormatter(std::make_shared<NetsimLogFormatter>("[%D %T.%F] %v"));

  // If you want to avoid showing the relative simulation time use
  // the native spdlog::pattern_formatter instead:
  // SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%D %T.%F]
  // %v"));
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));

  //---------------------------------------------------------------------

  // We recommend the callbacks to be very simple
  // since the ns3 simulation time is stopped during the ns3 callback
  // execution:
  // https://www.nsnam.org/docs/manual/html/realtime.html

  ns3::Config::Connect("/ROSDeviceList/*/PacketError",
                       ns3::MakeCallback(&HILNetSimTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&HILNetSimTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&HILNetSimTracing::PacketTransmitting, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxFifoSize",
      ns3::MakeCallback(&HILNetSimTracing::TxFifoUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/TxPacketDrops",
      ns3::MakeCallback(&HILNetSimTracing::PacketDropsUpdated, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacTx",
                       ns3::MakeCallback(&HILNetSimTracing::MacTx, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacRx",
                       ns3::MakeCallback(&HILNetSimTracing::MacRx, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxFifoSize",
      ns3::MakeCallback(&HILNetSimTracing::MacTxFifoUpdated, this));

  ns3::Config::Connect(
      "/NodeList/*/DeviceList/0/Mac/TxPacketDrops",
      ns3::MakeCallback(&HILNetSimTracing::MacPacketDropsUpdated, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/CourseChange",
      ns3::MakeCallback(&HILNetSimTracing::ShowPosition, this));

  freq = 10;
}

void HILNetSimTracing::GetLinearVel(const double &diffx, const double &diffy,
                                    const double &diffz, double &vx, double &vy,
                                    double &vz) {
  double kp = 0.1;
  double kp2 = 0.03;
  vx = diffx * kp;
  vy = diffy * kp;
  vz = diffz * kp;

  double mod = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (mod > 0.5) {
    vx = diffx * kp2;
    vy = diffy * kp2;
    vz = diffz * kp2;
  }
}

double HILNetSimTracing::GetAngularVel(const double &diff) {
  return diff * 0.1;
}

double HILNetSimTracing::GetExplorerAngularVel(const double &diff) {
  return diff * 0.8;
}

double HILNetSimTracing::AngleToRadians(const double &angle) {
  return angle * 2 * PI / 360;
}

void HILNetSimTracing::GetExplorerLinearVel(const double &diffx,
                                            const double &diffy,
                                            const double &diffz, double &vx,
                                            double &vy, double &vz) {
  double kp = 0.4;
  vx = diffx * kp;
  vy = diffy * kp;
  vz = diffz * kp;
}

void HILNetSimTracing::explorerTxWork(int src, CommsDeviceServicePtr &stream,
                                      std::mutex &mutex,
                                      const tf::Transform &wMeRef) {
  auto pb = dccomms::CreateObject<VariableLengthPacketBuilder>();
  auto pkt = pb->Create();
  auto pd = pkt->GetPayloadBuffer();
  int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
          *pitch = roll + 1, *yaw = pitch + 1;
  double tmp_roll, tmp_pitch, tmp_yaw;
  int dst;
  pkt->SetSrcAddr(src);
  if (acoustic)
    dst = 0;
  else if (src > 5)
    dst = 5;
  else
    dst = 1;

  pkt->SetDestAddr(dst);

  uint32_t seq = 0;
  uint32_t pdSize = 1 + 12;
  tf::Transform position;
  while (true) {
    mutex.lock();
    position = wMeRef;
    mutex.unlock();
    *x = position.getOrigin().x() * 100;
    *y = position.getOrigin().y() * 100;
    *z = position.getOrigin().z() * 100;
    position.getBasis().getRPY(tmp_roll, tmp_pitch, tmp_yaw);
    *roll = GetDiscreteRot(tmp_roll);
    *pitch = GetDiscreteRot(tmp_pitch);
    *yaw = GetDiscreteRot(tmp_yaw);

    pkt->SetSeq(seq++);
    pkt->PayloadUpdated(pdSize);
    stream << pkt;
    Info("E{}: TX TO {} SEQ {}", src, pkt->GetDestAddr(), pkt->GetSeq());

    std::this_thread::sleep_for(chrono::seconds(5));
  }
}

void HILNetSimTracing::explorerRxWork(int src, CommsDeviceServicePtr &stream,
                                      std::mutex &mutex,
                                      tf::Transform &wMl_comms,
                                      bool &received) {
  auto pb = dccomms::CreateObject<VariableLengthPacketBuilder>();
  auto pkt = pb->Create();
  auto pd = pkt->GetPayloadBuffer();
  int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
          *pitch = roll + 1, *yaw = pitch + 1;
  tf::Vector3 pos;
  double droll, dpitch, dyaw;
  tf::Quaternion rot;
  while (true) {
    stream >> pkt;
    if (pkt->PacketIsOk()) {
      // we have received the position of the leader
      pos = tf::Vector3(*x / 100., *y / 100., *z / 100.);
      droll = GetContinuousRot(*roll);
      dpitch = GetContinuousRot(*pitch);
      dyaw = GetContinuousRot(*yaw);
      rot = tf::createQuaternionFromRPY(droll, dpitch, dyaw);
      Info("E{}: RX FROM {} SEQ {} SIZE {} LP: {} {} {} {} {} {}", src,
           pkt->GetSrcAddr(), pkt->GetSeq(), pkt->GetPacketSize(), pos.x(),
           pos.y(), pos.z(), droll, dpitch, dyaw);

      mutex.lock();
      wMl_comms.setOrigin(pos);
      wMl_comms.setRotation(rot);
      received = true;
      mutex.unlock();
    }
  }
}

void HILNetSimTracing::DoRun() {

  std::thread gpsWork([&]() {
    tf::TransformListener listener;
    while (1) {
      try {

        std::unique_lock<std::mutex> wMe1_lock(wMe1_mutex);
        listener.lookupTransform("world", "explorer1", ros::Time(0), wMe1);
        wMe1_lock.unlock();

        std::unique_lock<std::mutex> wMe2_lock(wMe2_mutex);
        listener.lookupTransform("world", "explorer2", ros::Time(0), wMe2);
        wMe2_lock.unlock();

        std::unique_lock<std::mutex> wMe3_lock(wMe3_mutex);
        listener.lookupTransform("world", "explorer3", ros::Time(0), wMe3);
        wMe3_lock.unlock();

        std::unique_lock<std::mutex> lwMl1_lock(wMl1_mutex);
        listener.lookupTransform("world", "leader1", ros::Time(0), wMl1);
        lwMl1_lock.unlock();

        std::unique_lock<std::mutex> wMe1_2_lock(wMe1_2_mutex);
        listener.lookupTransform("world", "explorer1_2", ros::Time(0), wMe1_2);
        wMe1_2_lock.unlock();

        std::unique_lock<std::mutex> wMe2_2_lock(wMe2_2_mutex);
        listener.lookupTransform("world", "explorer2_2", ros::Time(0), wMe2_2);
        wMe2_2_lock.unlock();

        std::unique_lock<std::mutex> wMe3_2_lock(wMe3_2_mutex);
        listener.lookupTransform("world", "explorer3_2", ros::Time(0), wMe3_2);
        wMe3_2_lock.unlock();
        // std::this_thread::sleep_for(chrono::milliseconds(100));

      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        std::this_thread::sleep_for(chrono::milliseconds(100));
        continue;
      }
    }
  });
  gpsWork.detach();

  std::thread explorersWork([&]() {
    tf::TransformListener listener;
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    geometry_msgs::TwistStamped explorer_msg;

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "hil";
    static_transformStamped.child_frame_id = "te1";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "hil";
    static_transformStamped.child_frame_id = "te2";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "hil";
    static_transformStamped.child_frame_id = "te3";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "leader1";
    static_transformStamped.child_frame_id = "te1_2";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "leader1";
    static_transformStamped.child_frame_id = "te2_2";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = -1.5;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "leader1";
    static_transformStamped.child_frame_id = "te3_2";
    static_transformStamped.transform.translation.x = -1.5;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_transforms.push_back(static_transformStamped);

    static_broadcaster.sendTransform(static_transforms);
    ros::Rate rate(10);
    while (1) {
      try {
        listener.lookupTransform("hil", "te1", ros::Time(0), hilMte1);
        listener.lookupTransform("hil", "te2", ros::Time(0), hilMte2);
        listener.lookupTransform("hil", "te3", ros::Time(0), hilMte3);
        listener.lookupTransform("leader1", "te1_2", ros::Time(0), l1Mte1_2);
        listener.lookupTransform("leader1", "te2_2", ros::Time(0), l1Mte2_2);
        listener.lookupTransform("leader1", "te3_2", ros::Time(0), l1Mte3_2);
        break;
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        continue;
      }
    }
    while (1) {
      tf::StampedTransform e1Mte1_aux, e2Mte2_aux, e3Mte3_aux;
      while (1) {
        try {
          // tmp
          listener.lookupTransform("explorer1", "te1", ros::Time(0),
                                   e1Mte1_aux);
          listener.lookupTransform("explorer2", "te2", ros::Time(0),
                                   e2Mte2_aux);
          listener.lookupTransform("explorer3", "te3", ros::Time(0),
                                   e3Mte3_aux);
          break;
        } catch (tf::TransformException &ex) {
          Warn("TF: {}", ex.what());
          continue;
        }
      }
      // if (!wMhil_comms_received || !wMl1_comms_received || !wMtl1_received) {
      if (!wMl1_comms_received || !wMtl1_received) {
        std::this_thread::sleep_for(chrono::seconds(1));
        continue;
      }
      // Compute target transform from every slave in team 1 (input: hil
      // position from comms)
      wMhil_comms_mutex.lock();
      e1Mte1 = e1Mte1_aux; // wMe1.inverse() * wMhil_comms * hilMte1;
      e2Mte2 = e2Mte2_aux; // wMe2.inverse() * wMhil_comms * hilMte2;
      e3Mte3 = e3Mte3_aux; // wMe3.inverse() * wMhil_comms * hilMte3;
      wMhil_comms_mutex.unlock();

      // Compute target transform from every slave in team 2 (input: leader1
      // position from comms)
      wMl1_comms_mutex.lock();
      e1_2Mte1_2 = wMe1_2.inverse() * wMl1_comms * l1Mte1_2;
      e2_2Mte2_2 = wMe2_2.inverse() * wMl1_comms * l1Mte2_2;
      e3_2Mte3_2 = wMe3_2.inverse() * wMl1_comms * l1Mte3_2;
      wMl1_comms_mutex.unlock();

      // Compute target transform from leader1 (leader of team 2)
      wMtl1_comms_mutex.lock();
      l1Mtl1 = wMl1.inverse() * wMtl1_comms;
      wMtl1_comms_mutex.unlock();

      // Get Translations
      //    team 1
      auto e1Tte1 = e1Mte1.getOrigin();
      double e1x = e1Tte1.getX(), e1y = e1Tte1.getY(), e1z = e1Tte1.getZ();
      auto e2Tte2 = e2Mte2.getOrigin();
      double e2x = e2Tte2.getX(), e2y = e2Tte2.getY(), e2z = e2Tte2.getZ();
      auto e3Tte3 = e3Mte3.getOrigin();
      double e3x = e3Tte3.getX(), e3y = e3Tte3.getY(), e3z = e3Tte3.getZ();

      //    team 2
      auto e1_2Tte1_2 = e1_2Mte1_2.getOrigin();
      double e1_2x = e1_2Tte1_2.getX(), e1_2y = e1_2Tte1_2.getY(),
             e1_2z = e1_2Tte1_2.getZ();
      auto e2_2Tte2_2 = e2_2Mte2_2.getOrigin();
      double e2_2x = e2_2Tte2_2.getX(), e2_2y = e2_2Tte2_2.getY(),
             e2_2z = e2_2Tte2_2.getZ();
      auto e3_2Tte3_2 = e3_2Mte3_2.getOrigin();
      double e3_2x = e3_2Tte3_2.getX(), e3_2y = e3_2Tte3_2.getY(),
             e3_2z = e3_2Tte3_2.getZ();
      auto l1Ttl1 = l1Mtl1.getOrigin();
      double l1x = l1Ttl1.getX(), l1y = l1Ttl1.getY(), l1z = l1Ttl1.getZ();

      // Get Rotations
      //    team 1
      tfScalar roll1, pitch1, yaw1;
      auto mat1 = e1Mte1.getBasis();
      mat1.getRPY(roll1, pitch1, yaw1);
      tfScalar roll2, pitch2, yaw2;
      auto mat2 = e2Mte2.getBasis();
      mat2.getRPY(roll2, pitch2, yaw2);
      tfScalar roll3, pitch3, yaw3;
      auto mat3 = e3Mte3.getBasis();
      mat3.getRPY(roll3, pitch3, yaw3);

      //    team 2
      tfScalar roll1_2, pitch1_2, yaw1_2;
      auto mat1_2 = e1_2Mte1_2.getBasis();
      mat1_2.getRPY(roll1_2, pitch1_2, yaw1_2);
      tfScalar roll2_2, pitch2_2, yaw2_2;
      auto mat2_2 = e2_2Mte2_2.getBasis();
      mat2_2.getRPY(roll2_2, pitch2_2, yaw2_2);
      tfScalar roll3_2, pitch3_2, yaw3_2;
      auto mat3_2 = e3_2Mte3_2.getBasis();
      mat3_2.getRPY(roll3_2, pitch3_2, yaw3_2);
      tfScalar roll_l1, pitch_l1, yaw_l1;
      auto mat_l1 = l1Mtl1.getBasis();
      mat_l1.getRPY(roll_l1, pitch_l1, yaw_l1);

      double vx, vy, vz;
      tfScalar roll, pitch, yaw;

      // Compute and send velocities for TEAM 1
      GetExplorerLinearVel(e1x, e1y, e1z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll1);
      pitch = GetExplorerAngularVel(pitch1);
      yaw = GetExplorerAngularVel(yaw1);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e1_pub.publish(explorer_msg);

      GetExplorerLinearVel(e2x, e2y, e2z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll2);
      pitch = GetExplorerAngularVel(pitch2);
      yaw = GetExplorerAngularVel(yaw2);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e2_pub.publish(explorer_msg);

      GetExplorerLinearVel(e3x, e3y, e3z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll3);
      pitch = GetExplorerAngularVel(pitch3);
      yaw = GetExplorerAngularVel(yaw3);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e3_pub.publish(explorer_msg);

      // Compute and send velocities for TEAM 2
      GetExplorerLinearVel(e1_2x, e1_2y, e1_2z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll1_2);
      pitch = GetExplorerAngularVel(pitch1_2);
      yaw = GetExplorerAngularVel(yaw1_2);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e1_2pub.publish(explorer_msg);

      GetExplorerLinearVel(e2_2x, e2_2y, e2_2z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll2_2);
      pitch = GetExplorerAngularVel(pitch2_2);
      yaw = GetExplorerAngularVel(yaw2_2);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e2_2pub.publish(explorer_msg);

      GetExplorerLinearVel(e3_2x, e3_2y, e3_2z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll3_2);
      pitch = GetExplorerAngularVel(pitch3_2);
      yaw = GetExplorerAngularVel(yaw3_2);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      e3_2pub.publish(explorer_msg);

      GetExplorerLinearVel(l1x, l1y, l1z, vx, vy, vz);
      roll = GetExplorerAngularVel(roll_l1);
      pitch = GetExplorerAngularVel(pitch_l1);
      yaw = GetExplorerAngularVel(yaw_l1);

      explorer_msg.twist.linear.x = vx;
      explorer_msg.twist.linear.y = vy;
      explorer_msg.twist.linear.z = vz;
      explorer_msg.twist.angular.x = roll;
      explorer_msg.twist.angular.y = pitch;
      explorer_msg.twist.angular.z = yaw;

      leader1_pub.publish(explorer_msg);

      rate.sleep();
    }
  });
  explorersWork.detach();

  pb = dccomms::CreateObject<VariableLengthPacketBuilder>();

  // BUOY
  buoy = dccomms::CreateObject<CommsDeviceService>(pb);
  buoy->SetCommsDeviceId("comms_buoy");
  buoy->Start();

  std::thread buoyRxWork([&]() {
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    uint8_t *nrovs = pd;
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    while (true) {
      buoy >> pkt;
      if (pkt->PacketIsOk()) {
        uint32_t seq = pkt->GetSeq();
        Info("BUOY: RX FROM {} SEQ {} SIZE {}", pkt->GetSrcAddr(), seq,
             pkt->GetPacketSize());
      } else
        Warn("BUOY: ERR");
    }
  });

  std::thread buoyTxWork([&]() {
    // TEAM 2
    // Create TEAM 1 leader Waypoints
    std::vector<std::vector<double>> hilTargets;
    hilTargets.push_back(std::vector<double>{0.09, 0.42, 0.75, 360});
    hilTargets.push_back(std::vector<double>{-0.48, -0.24, 0.32, 1});
    hilTargets.push_back(std::vector<double>{-0.57, 0.4, 0.96, 1});
    hilTargets.push_back(std::vector<double>{-0.15, -0.18, 0.47, 7});
    hilTargets.push_back(std::vector<double>{0.08, 0.57, 0.47, 342});
    hilTargets.push_back(std::vector<double>{0.15, -0.32, 0.87, 4});
    hilTargets.push_back(std::vector<double>{0.09, 0.44, 0.8, 4});
    hilTargets.push_back(std::vector<double>{0.09, 0.44, 0.35, 4});
    hilTargets.push_back(std::vector<double>{0.15, -0.39, 0.35, 4});
    hilTargets.push_back(std::vector<double>{-0.22, 0.33, 0.44, 329});
    hilTargets.push_back(std::vector<double>{-0.23, 0.04, 0.44, 357});

    // Compute TEAM 2 Leader Waypoints
    int numWaypointsPerRad = 360;
    int numRads = 3;
    double radInc = 5;
    double rad = 10;
    double angle = 0;
    double angleInc = 360. / numWaypointsPerRad;
    tf::Transform e02Mte0;
    e02Mte0.setOrigin(tf::Vector3(0.75, 0.75, 0));
    e02Mte0.setRotation(tf::createQuaternionFromYaw(0));

    std::vector<tf::Transform> leaderTargetTfs;
    for (int r = 0; r < numRads; r++) {
      for (int i = 0; i < numWaypointsPerRad; i++) {
        tf::Transform eoMe02, eo2Mte0, rot, trans;
        double radians = AngleToRadians(angle);
        trans.setOrigin(tf::Vector3(0, rad, 0));
        trans.setRotation(tf::createQuaternionFromYaw(0));
        auto quat = tf::createQuaternionFromYaw(-radians).normalize();
        auto quatx = quat.x();
        auto quaty = quat.y();
        auto quatz = quat.z();
        auto quatw = quat.w();
        rot.setRotation(quat);
        rot.setOrigin(tf::Vector3(0, 0, 0));
        eoMe02 = rot * trans; // = explorers origin (leader0 in team0 / hil) to
                              // explorer leader target
        quat = eoMe02.getRotation();
        quatx = quat.x();
        quaty = quat.y();
        quatz = quat.z();
        quatw = quat.w();
        eo2Mte0 = eoMe02 * e02Mte0;
        angle += angleInc;
        leaderTargetTfs.push_back(eo2Mte0);
      }
      rad -= radInc;
    }

    auto pkt = pb->Create();
    pkt->SetSrcAddr(0);
    auto pd = pkt->GetPayloadBuffer();
    uint8_t *nrovs = pd;
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    double tmp_roll, tmp_pitch, tmp_yaw;

    uint32_t pdSize = 1 + 12;
    uint32_t l0seq = 0, l1seq = 0;

    int targetPositionIndex = 0;
    tf::Transform originMl1t, wMorigin, wMtl1;
    // World to NED = wMorigin
    wMorigin.setOrigin(tf::Vector3(0, 0, -10));
    std::vector<double> nextTarget;
    wMorigin.setRotation(tf::createQuaternionFromRPY(-M_PI, 0, 0));
    while (1) {
      uint idx = static_cast<uint>(targetPositionIndex);

      // Update leader0 position
      pkt->SetDestAddr(1);
      pkt->SetSeq(l0seq++);
      nextTarget = hilTargets[idx % hilTargets.size()];
      *x = nextTarget[0] * 100;
      *y = nextTarget[1] * 100;
      *z = nextTarget[2] * 100;
      *yaw = nextTarget[3];
      pkt->PayloadUpdated(pdSize);
      buoy << pkt;
      Info("BUOY: TX TO {} SEQ {}", pkt->GetDestAddr(), pkt->GetSeq());

      std::this_thread::sleep_for(chrono::seconds(0));

      // Update leader1 position
      originMl1t = leaderTargetTfs[idx % leaderTargetTfs.size()];
      wMtl1 = wMorigin * originMl1t;
      *x = wMtl1.getOrigin().x() * 100;
      *y = wMtl1.getOrigin().y() * 100;
      *z = wMtl1.getOrigin().z() * 100;
      wMtl1.getBasis().getRPY(tmp_roll, tmp_pitch, tmp_yaw);
      *roll = GetDiscreteRot(tmp_roll);
      *pitch = GetDiscreteRot(tmp_pitch);
      *yaw = GetDiscreteRot(tmp_yaw);
      pkt->SetDestAddr(5);
      pkt->SetSeq(l1seq++);
      pkt->PayloadUpdated(pdSize);
      buoy << pkt;
      Info("BUOY: TX TO {} SEQ {}", pkt->GetDestAddr(), pkt->GetSeq());

      std::this_thread::sleep_for(chrono::seconds(4));

      targetPositionIndex++;
    }
  });

  buoyRxWork.detach();
  buoyTxWork.detach();

  // TEAM 1
  hil = dccomms::CreateObject<CommsDeviceService>(pb);
  hil->SetCommsDeviceId("comms_hil");
  hil->Start();

  e1 = dccomms::CreateObject<CommsDeviceService>(pb);
  e1->SetCommsDeviceId("comms_explorer1");
  e1->Start();

  e2 = dccomms::CreateObject<CommsDeviceService>(pb);
  e2->SetCommsDeviceId("comms_explorer2");
  e2->Start();

  e3 = dccomms::CreateObject<CommsDeviceService>(pb);
  e3->SetCommsDeviceId("comms_explorer3");
  e3->Start();

  std::thread e1_tx_Work([&]() { explorerTxWork(2, e1, wMe1_mutex, wMe1); });
  std::thread e2_tx_Work([&]() { explorerTxWork(3, e2, wMe1_mutex, wMe1); });
  std::thread e3_tx_Work([&]() { explorerTxWork(4, e3, wMe2_mutex, wMe2); });
  std::thread e1_rx_Work([&]() {
    explorerRxWork(2, e1, wMhil_comms_mutex, wMhil_comms, wMhil_comms_received);
  });

  e1_tx_Work.detach();
  e2_tx_Work.detach();
  e3_tx_Work.detach();
  e1_rx_Work.detach();

  // TEAM 2
  leader1 = dccomms::CreateObject<CommsDeviceService>(pb);
  leader1->SetCommsDeviceId("comms_leader1");
  leader1->Start();

  e1_2 = dccomms::CreateObject<CommsDeviceService>(pb);
  e1_2->SetCommsDeviceId("comms_explorer1_2");
  e1_2->Start();

  e2_2 = dccomms::CreateObject<CommsDeviceService>(pb);
  e2_2->SetCommsDeviceId("comms_explorer2_2");
  e2_2->Start();

  e3_2 = dccomms::CreateObject<CommsDeviceService>(pb);
  e3_2->SetCommsDeviceId("comms_explorer3_2");
  e3_2->Start();

  std::thread leader1TxWork([&]() {
    auto pb = dccomms::CreateObject<VariableLengthPacketBuilder>();
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    double tmp_roll, tmp_pitch, tmp_yaw;
    int dst;
    pkt->SetSrcAddr(5);
    dst = 6;
    pkt->SetDestAddr(dst);

    uint32_t seq = 0;
    uint32_t pdSize = 1 + 12;
    tf::Transform position;
    while (true) {
      wMl1_mutex.lock();
      position = wMl1;
      wMl1_mutex.unlock();
      *x = position.getOrigin().x() * 100;
      *y = position.getOrigin().y() * 100;
      *z = position.getOrigin().z() * 100;
      position.getBasis().getRPY(tmp_roll, tmp_pitch, tmp_yaw);
      *roll = GetDiscreteRot(tmp_roll);
      *pitch = GetDiscreteRot(tmp_pitch);
      *yaw = GetDiscreteRot(tmp_yaw);

      pkt->SetSeq(seq++);
      pkt->PayloadUpdated(pdSize);
      leader1 << pkt;
      Info("L1: TX TO {} SEQ {}", pkt->GetDestAddr(), pkt->GetSeq(),
           pkt->GetSeq());

      std::this_thread::sleep_for(chrono::seconds(5));
    }
  });

  std::thread leader1RxWork([&]() {
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    tf::Vector3 pos;
    double droll, dpitch, dyaw;
    tf::Quaternion rot;
    while (true) {
      leader1 >> pkt;
      if (pkt->PacketIsOk()) {
        // we have received the desired position from the buoy
        pos = tf::Vector3(*x / 100., *y / 100., *z / 100.);
        droll = GetContinuousRot(*roll);
        dpitch = GetContinuousRot(*pitch);
        dyaw = GetContinuousRot(*yaw);
        rot = tf::createQuaternionFromRPY(droll, dpitch, dyaw);
        uint32_t seq = pkt->GetSeq();
        Info("L1: RX FROM {} SEQ {} SIZE {} REQ: {} {} {} {} {} {}",
             pkt->GetSrcAddr(), seq, pkt->GetPacketSize(), *x, *y, *z, *roll,
             *pitch, *yaw);
        wMtl1_comms_mutex.lock();
        wMtl1_comms.setOrigin(pos);
        wMtl1_comms.setRotation(rot);
        wMtl1_received = true;
        wMtl1_comms_mutex.unlock();
      } else
        Warn("L1: ERR");
    }
  });

  std::thread e1_2_tx_Work(
      [&]() { explorerTxWork(6, e1_2, wMe1_2_mutex, wMe1_2); });
  std::thread e2_2_tx_Work(
      [&]() { explorerTxWork(7, e2_2, wMe1_2_mutex, wMe1_2); });
  std::thread e3_2_tx_Work(
      [&]() { explorerTxWork(8, e3_2, wMe2_2_mutex, wMe2_2); });
  std::thread e1_2_rx_Work([&]() {
    explorerRxWork(6, e1_2, wMl1_comms_mutex, wMl1_comms, wMl1_comms_received);
  });

  leader1RxWork.detach();
  leader1TxWork.detach();
  e1_2_tx_Work.detach();
  e2_2_tx_Work.detach();
  e3_2_tx_Work.detach();
  e1_2_rx_Work.detach();
}

CLASS_LOADER_REGISTER_CLASS(HILNetSimTracing, NetSimTracing)
} // namespace sensors_si2019
