#include <dccomms_packets/VariableLengthPacket.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <underwater_sensor_msgs/LedLight.h>
#include <uwsim/NetSim.h>

namespace sensors_si2019 {

using namespace uwsim;
using namespace dccomms_packets;
using namespace dccomms;

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

  dccomms::Ptr<VariableLengthPacketBuilder> pb;
  dccomms::Ptr<CommsDeviceService> buoy, hil, leader1, e1, e2, e3, e1_2, e2_2,
      e3_2;

  std::mutex wMe1_mutex, wMe2_mutex, wMe3_mutex, wMl1_mutex, wMe0_2_mutex, wMe1_2_mutex,
      wMe2_2_mutex, wMe3_2_mutex, wMhil_comms_mutex, wMl1_comms_mutex, wMtl1_comms_mutex;
  tf::Transform e1Mte1, e2Mte2, e3Mte3, e1_2Mte1_2, e2_2Mte2_2, e3_2Mte3_2, wMtl1_comms,
      l1Mtl1;
  tf::StampedTransform hilMte1, hilMte2, hilMte3, l1Mte1_2, l1Mte2_2, l1Mte3_2,
      wMe1, wMe2, wMe3, wMl1, wMe1_2, wMe2_2, wMe3_2;

  tf::Transform wMhil_comms, wMl1_comms;
  bool acoustic = true;
  bool wMhil_comms_received = false, wMl1_comms_received = false, wMtl1_received = false;
  void explorerTxWork(int src, CommsDeviceServicePtr &stream, std::mutex &mutex,
                      const tf::Transform &wMeRef);
  void explorerRxWork(int src, CommsDeviceServicePtr &stream, std::mutex &mutex,
                      tf::Transform &wMl_comms, bool &received);
};
} // namespace uwsim_netstim
