#include <chrono>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/Utils.h>
#include <dccomms_packets/VariableLengthPacket.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCS.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensors_si2019/Constants.h>
#include <sensors_si2019/pid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <umci/DcMac.h>
#include <wireless_ardusub/OperatorController.h>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;
using namespace sensors_si2019;
using namespace wireless_ardusub;

int main(int argc, char **argv) {
  auto log = CreateLogger("Main");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  log->LogToConsole(true);
  log->SetAsyncMode();

  ros::init(argc, argv, "hil_leader");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  OperatorController::Params params;
  params.sitl = true;

  OperatorController op(params);
  op.Start();

  while (1) {
    ros::spinOnce();
    rate.sleep();
  }
}
