#include <dccomms_packets/VariableLength2BPacket.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <umci/DcMac.h>
#include <underwater_sensor_msgs/LedLight.h>
#include <uwsim/NetSim.h>
#include <mavlink_ros/OperatorController.h>

namespace sensors_si2019 {

using namespace uwsim;
using namespace dccomms_packets;
using namespace dccomms;
using namespace umci;
using namespace mavlink_ros;

class HILNetSimTracing : public NetSimTracing {
public:
  HILNetSimTracing();

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3ConstPacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev,
                   ns3ConstPacketPtr pkt, bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);

  void TxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void PacketDropsUpdated(std::string path, uint32_t oldValue,
                          uint32_t newValue);

  void MacTxFifoUpdated(std::string path, uint32_t oldValue, uint32_t newValue);
  void MacPacketDropsUpdated(std::string path, uint32_t oldValue,
                             uint32_t newValue);

  void MacRx(std::string path, ROSCommsDevicePtr dev, ns3ConstPacketPtr pkt);
  void MacTx(std::string path, ROSCommsDevicePtr dev, ns3ConstPacketPtr pkt);

  void LeaderOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
  void FollowerOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
  void SupportOdomCb(const nav_msgs::Odometry::ConstPtr &msg);

  void Configure();
  void DoRun();

  void ShowPosition(string path, ROSCommsDevicePtr dev, const tf::Vector3 &pos);

  ros::NodeHandle node;
  ros::Publisher leader1_pub, e1_pub, e2_pub, e3_pub, e1_2pub, e2_2pub, e3_2pub;

  struct PIDStatus {};
  PIDStatus leader_pid, follower_pid, support_pid;
  int freq;
  void GetLinearVel(const double &diffx, const double &diffy,
                    const double &diffz, double &vx, double &vy, double &vz);

  void GetExplorerLinearVel(const double &diffx, const double &diffy,
                            const double &diffz, double &vx, double &vy,
                            double &vz);
  double GetAngularVel(const double &diff);
  double GetExplorerAngularVel(const double &diff);

  double AngleToRadians(const double &angle);
  CommsDeviceServicePtr ConfigureDcMacLayerOnExplorer(const int &addr);
  CommsDeviceServicePtr ConfigureDcMacLayerOnLeader(const int &addr,
                                                    const bool &rf);
  CommsDeviceServicePtr ConfigureDcMacLayerOnBuoy(const int &addr = 0);

  dccomms::Ptr<VariableLength2BPacketBuilder> pb;
  dccomms::CommsDeviceServicePtr buoy, hil_large, hil_short, hil_rf, hil_ac, e1,
      e2, e3;

  std::mutex wMe1_mutex, wMe2_mutex, wMe3_mutex, wMhil_mutex, wMhil_comms_mutex,
      wMthil_comms_mutex, wMthil_mutex;
  tf::Transform e1Mte1_comms, e2Mte2_comms, e3Mte3_comms, wMthil_comms, wMthil,
      e1Mte1, e2Mte2, e3Mte3;
  tf::StampedTransform hilMte1, hilMte2, hilMte3, wMhil, wMe1, wMe2, wMe3;

  tf::Transform wMhil_comms;
  bool wMhil_comms_received = false, wMthil_comms_received = false;
  void explorerTxWork(int src, int dst, CommsDeviceServicePtr &stream,
                      std::mutex &mutex, const tf::Transform &wMeRef);
  void explorerRxWork(int src, CommsDeviceServicePtr &stream, std::mutex &mutex,
                      tf::Transform &wMl_comms, bool &received);

  int16_t buoy_addr, hil_ac_addr, e1_addr, e2_addr, e3_addr, hil_rf_addr,
      t0_dst;
  OperatorController::Params params;
  dccomms::Ptr<OperatorController> op;
  bool initPosReached = false;
  bool wMhil_updated = false;
  bool wMthil_updated = false;

  void TestExit();

  void ArmHIL();

protected:
  bool use_rf_channels;
  bool use_umci_mac;
  uint32_t buoyPacketSize = 50;
  uint32_t hilPacketSize = 50;
  uint32_t explorersPacketSize = 600;
  uint32_t explorersDataRate = 300;
  uint32_t buoyDataRate = 100;
  uint32_t hilDataRate = 100;
  uint64_t desiredPositionUpdateIntervalMillis = 30000;

  double ac_maxDistance = 100;
  double rf_maxDistance = 5;
  uint32_t rf_bitrate = 1800;
  uint32_t ac_bitrate = 1800;
  uint32_t rf_nnodes = 4;
  uint32_t ac_nnodes = 5;
  cpplogging::LogLevel dcmacLogLevel = off;
  uint32_t derrorLogIntervalMillis = 250;
  std::string hiltfname = "erov"; // aruco_mapping filtered
};

// The following class can only be used when using the ArduSub SITL simulator
// and its FDM output
class UWSimTF_HILNetSimTracing : public HILNetSimTracing {
public:
  UWSimTF_HILNetSimTracing() : HILNetSimTracing() {}
  void Configure() {
    hiltfname = "rov"; // UWSim TF
    HILNetSimTracing::Configure();
  }
};

class UMCIMAC_HILNetSimTracing : public HILNetSimTracing {
public:
  UMCIMAC_HILNetSimTracing() : HILNetSimTracing() {}
  void Configure() {
    use_umci_mac = true;
    HILNetSimTracing::Configure();
  }
};

class UWSimTF_UMCIMAC_HILNetSimTracing : public HILNetSimTracing {
public:
  UWSimTF_UMCIMAC_HILNetSimTracing() : HILNetSimTracing() {}
  void Configure() {
    use_umci_mac = true;
    hiltfname = "rov"; // UWSim TF
    HILNetSimTracing::Configure();
  }
};

} // namespace sensors_si2019
