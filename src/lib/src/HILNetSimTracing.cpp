#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensors_si2019/Constants.h>
#include <sensors_si2019/HILNetSimTracing.h>
#include <sensors_si2019/utils.hpp>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <umci/DcMac.h>

namespace sensors_si2019 {

using namespace dccomms_packets;
using namespace utils;
using namespace umci;
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

  pb = dccomms::CreateObject<VariableLength2BPacketBuilder>();
  use_rf_channels = false;
  use_umci_mac = false;
  params.sitl = true;
  op = dccomms::CreateObject<OperatorController>(params);
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

void HILNetSimTracing::explorerTxWork(int src, int dst,
                                      CommsDeviceServicePtr &stream,
                                      std::mutex &mutex,
                                      const tf::Transform &wMeRef) {
  auto pb = dccomms::CreateObject<VariableLength2BPacketBuilder>();
  auto pkt = pb->Create();
  auto pd = pkt->GetPayloadBuffer();
  int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
          *pitch = roll + 1, *yaw = pitch + 1;
  double tmp_roll, tmp_pitch, tmp_yaw;
  pkt->SetSrcAddr(src);
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
  auto pb = dccomms::CreateObject<VariableLength2BPacketBuilder>();
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

      //      mutex.lock();
      //      wMl_comms.setOrigin(pos);
      //      wMl_comms.setRotation(rot);
      //      received = true;
      //      mutex.unlock();
    }
  }
}

CommsDeviceServicePtr
HILNetSimTracing::ConfigureDcMacLayerOnBuoy(const int &addr) {
  auto dcmac = dccomms::CreateObject<DcMac>();
  dcmac->SetPktBuilder(pb);
  dcmac->SetAddr(addr);
  dcmac->SetDevBitRate(6900);
  dcmac->SetDevIntrinsicDelay(1);
  dcmac->SetMaxDistance(100);
  dcmac->SetPropSpeed(1500);
  dcmac->SetMaxDataSlotDur(1000);
  dcmac->UpdateSlotDurFromEstimation();
  dcmac->SetMode(DcMac::Mode::master);
  if (use_rf_channels) {
    dcmac->SetNumberOfNodes(4);
  } else {
    dcmac->SetNumberOfNodes(9);
  }
}
CommsDeviceServicePtr
HILNetSimTracing::ConfigureDcMacLayerOnExplorer(const int &addr) {
  auto dcmac = dccomms::CreateObject<DcMac>();
  if (use_rf_channels) {
    dcmac->SetPktBuilder(pb);
    dcmac->SetAddr(addr);
    dcmac->SetDevBitRate(1900);
    dcmac->SetDevIntrinsicDelay(1);
    dcmac->SetMaxDistance(5);
    dcmac->SetPropSpeed(3e8);
    dcmac->SetNumberOfNodes(4);
    dcmac->SetMaxDataSlotDur(1000);
    dcmac->UpdateSlotDurFromEstimation();
    dcmac->SetMode(DcMac::Mode::slave);
  } else {
    dcmac->SetPktBuilder(pb);
    dcmac->SetAddr(addr);
    dcmac->SetDevBitRate(6900);
    dcmac->SetDevIntrinsicDelay(1);
    dcmac->SetMaxDistance(100);
    dcmac->SetPropSpeed(1500);
    dcmac->SetNumberOfNodes(9);
    dcmac->SetMaxDataSlotDur(1000);
    dcmac->UpdateSlotDurFromEstimation();
    dcmac->SetMode(DcMac::Mode::slave);
  }
}

CommsDeviceServicePtr
HILNetSimTracing::ConfigureDcMacLayerOnLeader(const int &addr, const bool &rf) {
  auto dcmac = dccomms::CreateObject<DcMac>();
  if (rf) {
    dcmac->SetPktBuilder(pb);
    dcmac->SetAddr(addr);
    dcmac->SetDevBitRate(1900);
    dcmac->SetDevIntrinsicDelay(1);
    dcmac->SetMaxDistance(5);
    dcmac->SetPropSpeed(3e8);
    dcmac->SetNumberOfNodes(4);
    dcmac->SetMaxDataSlotDur(1000);
    dcmac->UpdateSlotDurFromEstimation();
    dcmac->SetMode(DcMac::Mode::master);
  } else {
    dcmac->SetPktBuilder(pb);
    dcmac->SetAddr(addr);
    dcmac->SetDevBitRate(6900);
    dcmac->SetDevIntrinsicDelay(1);
    dcmac->SetMaxDistance(100);
    dcmac->SetPropSpeed(1500);
    dcmac->SetNumberOfNodes(9);
    dcmac->SetMaxDataSlotDur(1000);
    dcmac->UpdateSlotDurFromEstimation();
    dcmac->SetMode(DcMac::Mode::master);
  }
}

void HILNetSimTracing::DoRun() {

  op->SetReferenceTfName("local_origin_ned");

  op->ControlState.mode = FLY_MODE_R::MANUAL;
  op->ControlState.arm = false;
  op->ControlState.x = 0;
  op->ControlState.y = 0;
  op->ControlState.z = 0;
  op->ControlState.r = 0;

  op->Start();

  std::thread reachInitPos([&]() {
    tf::TransformBroadcaster sender;
    tf::TransformListener listener;
    tf::StampedTransform hilMinit;
    op->SetRobotTfName("hil");
    op->SetDesiredPosTfName("hil_init");

    tf::Vector3 origin(0, 0, 0);
    while (1) {
      try {
        listener.lookupTransform("hil", "hil_init", ros::Time(0), hilMinit);
        if (hilMinit.getOrigin().distance(origin) > 0.2) {
          op->ControlState.mode = FLY_MODE_R::GUIDED;
          op->ControlState.arm = true;
        } else {
          initPosReached = true;
          op->SetDesiredPosTfName("hil_target");
          op->ResetPID();
          break;
        }

      } catch (tf::TransformException &ex) {
        op->ResetPID();
        Warn("TF: {}", ex.what());
        std::this_thread::sleep_for(chrono::milliseconds(100));
        continue;
      }
    }
  });
  reachInitPos.detach();

  std::thread gpsWork([&]() {
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    geometry_msgs::TransformStamped static_transformStamped;
    tf::TransformListener listener;
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "local_origin_ned";
    static_transformStamped.child_frame_id = "hil_init";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0.85;
    static_transformStamped.transform.rotation.x = rotation.getX();
    static_transformStamped.transform.rotation.y = rotation.getY();
    static_transformStamped.transform.rotation.z = rotation.getZ();
    static_transformStamped.transform.rotation.w = rotation.getW();
    static_transforms.push_back(static_transformStamped);
    static_broadcaster.sendTransform(static_transforms);

    while (1) {
      try {
        std::unique_lock<std::mutex> wMhil_lock(wMhil_mutex);
        listener.lookupTransform("local_origin_ned", "hil", ros::Time(0),
                                 wMhil);
        wMhil_lock.unlock();

        std::unique_lock<std::mutex> wMe1_lock(wMe1_mutex);
        listener.lookupTransform("local_origin_ned", "explorer1", ros::Time(0),
                                 wMe1);
        wMe1_lock.unlock();

        std::unique_lock<std::mutex> wMe2_lock(wMe2_mutex);
        listener.lookupTransform("local_origin_ned", "explorer2", ros::Time(0),
                                 wMe2);
        wMe2_lock.unlock();

        std::unique_lock<std::mutex> wMe3_lock(wMe3_mutex);
        listener.lookupTransform("local_origin_ned", "explorer3", ros::Time(0),
                                 wMe3);
        wMe3_lock.unlock();

      } catch (tf::TransformException &ex) {
        op->ResetPID();
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
    tf::TransformBroadcaster broadcaster;

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);

    hilMte1.setOrigin(tf::Vector3(0, -1.5, 0));
    hilMte2.setOrigin(tf::Vector3(-1.5, -1.5, 0));
    hilMte3.setOrigin(tf::Vector3(-1.5, 0, 0));

    hilMte1.setRotation(rotation);
    hilMte2.setRotation(rotation);
    hilMte3.setRotation(rotation);

    ros::Rate rate(10);
    while (1) {
      if (!wMhil_comms_received || !wMthil_comms_received) {
        std::this_thread::sleep_for(chrono::seconds(1));
        continue;
      }
      if (wMthil_comms_received) {
        op->ControlState.mode = FLY_MODE_R::GUIDED;
        try {
          broadcaster.sendTransform(
              tf::StampedTransform(wMthil_comms, ros::Time::now(),
                                   "local_origin_ned", "hil_target"));

        } catch (tf::TransformException &ex) {
          op->ResetPID();
          Warn("TF: {}", ex.what());
          std::this_thread::sleep_for(chrono::milliseconds(100));
        }
      }

      // Compute target transform from every slave in team 1 (input: hil
      // position from comms)
      wMe1_mutex.lock();
      wMe2_mutex.lock();
      wMe3_mutex.lock();
      wMhil_comms_mutex.lock();
      e1Mte1 = wMe1.inverse() * wMhil_comms * hilMte1;
      e2Mte2 = wMe2.inverse() * wMhil_comms * hilMte2;
      e3Mte3 = wMe3.inverse() * wMhil_comms * hilMte3;
      wMhil_comms_mutex.unlock();
      wMe1_mutex.unlock();
      wMe2_mutex.unlock();
      wMe3_mutex.unlock();

      // Get Translations
      //    team 1
      auto e1Tte1 = e1Mte1.getOrigin();
      double e1x = e1Tte1.getX(), e1y = e1Tte1.getY(), e1z = e1Tte1.getZ();
      auto e2Tte2 = e2Mte2.getOrigin();
      double e2x = e2Tte2.getX(), e2y = e2Tte2.getY(), e2z = e2Tte2.getZ();
      auto e3Tte3 = e3Mte3.getOrigin();
      double e3x = e3Tte3.getX(), e3y = e3Tte3.getY(), e3z = e3Tte3.getZ();

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

      rate.sleep();
    }
  });
  explorersWork.detach();

  buoy_addr = 0;

  if (use_umci_mac) {

    // Si usamos una capa mac externa a UWSim-NET podemos repetir las mac. No
    // obstante, en el xml estas mac han de ser diferente para no producir
    // error.

    if (use_rf_channels) {
      hil_ac_addr = 1;
      hil_rf_addr = 0;
      e1_addr = 1;
      e2_addr = 2;
      e3_addr = 3;

      t0_dst = hil_rf_addr;

    } else {
      hil_ac_addr = 1;
      e1_addr = 2;
      e2_addr = 3;
      e3_addr = 4;

      t0_dst = buoy_addr;
    }

    buoy = ConfigureDcMacLayerOnBuoy();
    hil_ac = ConfigureDcMacLayerOnLeader(hil_ac_addr, false);
    e1 = ConfigureDcMacLayerOnExplorer(e1_addr);
    e2 = ConfigureDcMacLayerOnExplorer(e2_addr);
    e3 = ConfigureDcMacLayerOnExplorer(e3_addr);

  } else {

    // Si usamos una capa mac implementada en UWSim-NET las mac han de ser
    // diferentes para cada dispositivo. TODO: permitir repeticion de macs en
    // UWSim-NET
    hil_ac_addr = 1;
    hil_rf_addr = 2;
    e1_addr = 3;
    e2_addr = 4;
    e3_addr = 5;

    if (use_rf_channels) {
      t0_dst = hil_rf_addr;

      hil_rf = dccomms::CreateObject<CommsDeviceService>(pb);

    } else {
      t0_dst = buoy_addr;
    }

    buoy = dccomms::CreateObject<CommsDeviceService>(pb);
    hil_ac = dccomms::CreateObject<CommsDeviceService>(pb);
    e1 = dccomms::CreateObject<CommsDeviceService>(pb);
    e2 = dccomms::CreateObject<CommsDeviceService>(pb);
    e3 = dccomms::CreateObject<CommsDeviceService>(pb);
  }

  if (use_rf_channels) {
    hil_rf->SetCommsDeviceId("comms_hil_rf");
    hil_rf->Start();
  }

  hil_ac->SetCommsDeviceId("comms_hil_ac");
  hil_ac->Start();

  buoy->SetCommsDeviceId("comms_buoy");
  buoy->Start();
  e1->SetCommsDeviceId("comms_explorer1");
  e1->Start();
  e2->SetCommsDeviceId("comms_explorer2");
  e2->Start();
  e3->SetCommsDeviceId("comms_explorer3");
  e3->Start();

  std::thread buoyRxWork([&]() {
    auto pb = dccomms::CreateObject<VariableLength2BPacketBuilder>();
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    tf::Vector3 pos;
    double droll, dpitch, dyaw;
    tf::Quaternion rot;
    tf::TransformBroadcaster tf_broadcaster;
    while (true) {
      buoy >> pkt;
      if (pkt->PacketIsOk()) {
        // we have received the position of the leader
        pos = tf::Vector3(*x / 100., *y / 100., *z / 100.);
        droll = GetContinuousRot(*roll);
        dpitch = GetContinuousRot(*pitch);
        dyaw = GetContinuousRot(*yaw);
        rot = tf::createQuaternionFromRPY(droll, dpitch, dyaw);
        Info("BUOY: RX FROM {} SEQ {} SIZE {} LP: {} {} {} {} {} {}",
             pkt->GetSrcAddr(), pkt->GetSeq(), pkt->GetPacketSize(), pos.x(),
             pos.y(), pos.z(), droll, dpitch, dyaw);

        if (pkt->GetSrcAddr() == hil_ac_addr) {
          wMhil_comms_mutex.lock();
          wMhil_comms.setOrigin(pos);
          wMhil_comms.setRotation(rot);
          wMhil_comms_received = true;
          wMhil_comms_mutex.unlock();
        }
      }
    }
  });

  std::thread buoyTxWork([&]() {
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
    std::vector<double> nextTarget;
    while (1) {
      if (!initPosReached) {
        std::this_thread::sleep_for(chrono::seconds(1));
        continue;
      }

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

      std::this_thread::sleep_for(chrono::seconds(4));

      targetPositionIndex++;
    }
  });

  buoyRxWork.detach();
  buoyTxWork.detach();

  // TEAM 1

  std::thread e1_tx_Work(
      [&]() { explorerTxWork(e1_addr, t0_dst, e1, wMe1_mutex, wMe1); });
  std::thread e2_tx_Work(
      [&]() { explorerTxWork(e2_addr, t0_dst, e2, wMe1_mutex, wMe1); });
  std::thread e3_tx_Work(
      [&]() { explorerTxWork(e3_addr, t0_dst, e3, wMe2_mutex, wMe2); });
  std::thread e1_rx_Work([&]() {
    explorerRxWork(e1_addr, e1, wMhil_comms_mutex, wMhil_comms,
                   wMhil_comms_received);
  });

  e1_tx_Work.detach();
  e2_tx_Work.detach();
  e3_tx_Work.detach();
  e1_rx_Work.detach();

  if (use_rf_channels) {
    hil_large = hil_ac;
    hil_short = hil_rf;
  } else {
    hil_large = hil_ac;
    hil_short = hil_ac;
  }
  std::thread hilTxWork([&]() {
    auto pb = dccomms::CreateObject<VariableLength2BPacketBuilder>();
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    double tmp_roll, tmp_pitch, tmp_yaw;
    pkt->SetSrcAddr(hil_ac_addr);

    uint32_t seq = 0;
    uint32_t pdSize = 1 + 12;
    tf::Transform position;
    while (true) {
      wMhil_mutex.lock();
      position = wMhil;
      wMhil_mutex.unlock();
      *x = position.getOrigin().x() * 100;
      *y = position.getOrigin().y() * 100;
      *z = position.getOrigin().z() * 100;
      position.getBasis().getRPY(tmp_roll, tmp_pitch, tmp_yaw);
      *roll = GetDiscreteRot(tmp_roll);
      *pitch = GetDiscreteRot(tmp_pitch);
      *yaw = GetDiscreteRot(tmp_yaw);

      pkt->SetSeq(seq++);
      pkt->SetDestAddr(buoy_addr);
      pkt->PayloadUpdated(pdSize);

      hil_large << pkt;
      Info("HIL: TX TO {} SEQ {}", pkt->GetDestAddr(), pkt->GetSeq(),
           pkt->GetSeq());

      std::this_thread::sleep_for(chrono::seconds(5));
    }
  });
  hilTxWork.detach();

  std::thread hilRxWork([&]() {
    auto pkt = pb->Create();
    auto pd = pkt->GetPayloadBuffer();
    int16_t *x = (int16_t *)(pd + 1), *y = x + 1, *z = y + 1, *roll = z + 1,
            *pitch = roll + 1, *yaw = pitch + 1;
    tf::Vector3 pos;
    double droll, dpitch, dyaw;
    tf::Quaternion rot;
    while (true) {
      hil_large >> pkt;
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
        wMthil_comms_mutex.lock();
        wMthil_comms.setOrigin(pos);
        wMthil_comms.setRotation(rot);
        wMthil_comms_received = true;
        wMthil_comms_mutex.unlock();
      } else
        Warn("L1: ERR");
    }
  });
  hilRxWork.detach();
}

CLASS_LOADER_REGISTER_CLASS(HILNetSimTracing, NetSimTracing)
CLASS_LOADER_REGISTER_CLASS(RF_HILNetSimTracing, NetSimTracing)
CLASS_LOADER_REGISTER_CLASS(RF_UMCIMAC_HILNetSimTracing, NetSimTracing)
CLASS_LOADER_REGISTER_CLASS(UMCIMAC_HILNetSimTracing, NetSimTracing)
} // namespace sensors_si2019
