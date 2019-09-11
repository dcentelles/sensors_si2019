#include <chrono>
#include <dccomms/Utils.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensors_si2019/Constants.h>
#include <sensors_si2019/pid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <dccomms/CommsDeviceService.h>
#include <dccomms_packets/VariableLengthPacket.h>
#include <umci/DcMac.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;
using namespace sensors_si2019;

class OperatorController : public Logger {
public:
  OperatorController(ros::NodeHandle &nh);
  void Start();

private:
  struct ControlState {
    double x, y, z, r;
    FLY_MODE_R mode;
    bool arm;
  };
  ControlState _controlState;
  // ATTRIBUTES
  ros::NodeHandle &_nh;
  ros::Subscriber _joy_sub;
  std::vector<int> _previous_buttons;

  tf::TransformListener listener;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform nedMerov, nedMtarget, cameraMrov;
  tf::Transform rovMtarget;
  tf::Transform rovMned;
  std::shared_ptr<GCSv1> control;
  dccomms::Timer timer;

  double vmax = 1000, vmin = -1000;
  sensors_si2019::PID yawPID = sensors_si2019::PID(vmax, vmin, 10, 20, 0.05),
                      xPID = sensors_si2019::PID(vmax, vmin, 15, 60, 0.05),
                      yPID = sensors_si2019::PID(vmax, vmin, 15, 60, 0.05),
                      zPID = sensors_si2019::PID(vmax, vmin, 20, 10, 0.05);


  double keepHeadingIteration(const double &dt, double diff);
  double ArduSubXYR(double per);
  double ArduSubZ(double per);
  void Saturate(const double &max, const double &x, const double &y,
                const double &z, double &vx, double &vy, double &vz);
  void GetLinearXVel(const double &dt, const double &diff, double &v);
  void GetLinearYVel(const double &dt, const double &diff, double &v);
  void GetLinearZVel(const double &dt, const double &diff, double &v);
  void Loop();
  void ResetPID();

  std::thread _mainLoop;
};

OperatorController::OperatorController(ros::NodeHandle &nh) : _nh(nh) {
  SetLogName("GCS");
  SetLogLevel(info);
  SetAsyncMode();
  FlushLogOn(debug);
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
  SetAsyncMode(true);
  uint16_t localPort = 14550;
  control = std::shared_ptr<GCSv1>(new GCSv1(localPort));
  control->SetLogName("GCS");
  control->SetLogLevel(info);

  control->EnableGPSMock(false);
  control->SetManualControl(0, 0, 0, 0);
  control->EnableManualControl(true);
  control->Start();

  _controlState.arm = false;
  _controlState.mode = MANUAL;
}

void OperatorController::Start() {
  _mainLoop = std::thread([this]() { Loop(); });
}

void OperatorController::ResetPID() {
  yawPID.Reset();
  xPID.Reset();
  yPID.Reset();
  zPID.Reset();
  timer.Reset();
}

void OperatorController::Loop() {
  bool manual = true;

  double tyaw, cyaw;
  while (ros::ok()) {
    if (manual) {
      ResetPID();
      manual = false;
    }
    try {
      listener.lookupTransform("local_origin_ned", "erov", ros::Time(0),
                               nedMerov);
      listener.lookupTransform("local_origin_ned", "bluerov2_ghost",
                               ros::Time(0), nedMtarget);
    } catch (tf::TransformException &ex) {
      Warn("TF: {}", ex.what());
      control->Arm(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    control->SetFlyMode(FLY_MODE_R::STABILIZE);
    control->Arm(true);

    tf::Vector3 nedTerov = nedMerov.getOrigin();
    cyaw = tf::getYaw(nedMerov.getRotation());

    tf::Vector3 nedTtarget = nedMtarget.getOrigin();
    tyaw = tf::getYaw(nedMtarget.getRotation());

    rovMtarget = nedMerov.inverse() * nedMtarget;

    tf::Vector3 erovTtarget = rovMtarget.getOrigin();
    tf::Quaternion erovRtarget = rovMtarget.getRotation();

    double vx, vy, vz;
    double vTlpX = erovTtarget.getX(), vTlpY = erovTtarget.getY(),
           vTlpZ = erovTtarget.getZ();

    double elapsedSecs = timer.Elapsed() / 1000.;

    GetLinearXVel(elapsedSecs, vTlpX, vx);
    GetLinearYVel(elapsedSecs, vTlpY, vy);
    GetLinearZVel(elapsedSecs, -vTlpZ, vz);

    if (vx > 100)
      vx = 100;
    if (vx < -100)
      vx = -100;
    if (vy > 100)
      vy = 100;
    if (vy < -100)
      vy = -100;
    if (vz > 100)
      vz = 100;
    if (vz < -100)
      vz = -100;

    double rdiff = tf::getYaw(erovRtarget);
    double rv0 = keepHeadingIteration(elapsedSecs, rdiff);

    double baseZ = 0; //-38;
    double newZ = vz + baseZ;
    auto x = ceil(ArduSubXYR(vx));
    auto y = ceil(ArduSubXYR(vy));
    auto z = ceil(ArduSubZ(newZ));
    auto r = ceil(ArduSubXYR(rv0));

    double yoffset = 90;
    double xoffset = 90;
    double roffset = 460;
    double zoffset = 10;
    double deadband = 0;

    if (y > deadband)
      y += yoffset;
    else if (y < -deadband)
      y -= yoffset;

    if (x > deadband)
      x += xoffset;
    else if (x < -deadband)
      x -= xoffset;

    if (z > 500)
      z += 150;
    else if (z < 500)
      z -= zoffset;

    if (r > deadband)
      r += roffset + 5;
    else if (r < -deadband)
      r -= roffset;

    Info("Send order: X: {} ({}) ; Y: {} ({}) ; Z: {} ({}) ; R: {} ;  rdiff: "
         "{} ; rout: {} "
         "; rinput: {} ; Arm: {}",
         x, vx, y, vy, z, vz, r, rdiff, rv0, rv0,
         _controlState.arm ? "true" : "false");

    control->SetManualControl(x, y, z, r);
    timer.Reset();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// from -100,100 to -1000,1000
double OperatorController::ArduSubXYR(double per) { return per * 10; }
double OperatorController::ArduSubZ(double per) { return (per + 100) / 0.2; }

double OperatorController::keepHeadingIteration(const double &dt, double diff) {
  double vel = yawPID.calculate(dt, 0, -1 * diff);

  return vel;
}

void OperatorController::Saturate(const double &max, const double &x,
                                  const double &y, const double &z, double &vx,
                                  double &vy, double &vz) {
  double rx, ry, rz;
  rx = x / max;
  ry = y / max;
  rz = y / max;
  if (rx > 1 || ry > 1 || rz > 1) {
    double alpha = rx;
    if (alpha < ry)
      alpha = ry;
    if (alpha < rz)
      alpha = rz;
    vx = x / alpha;
    vy = y / alpha;
    vz = z / alpha;
  } else {
    vx = x;
    vy = y;
    vz = z;
  }
}

void OperatorController::GetLinearXVel(const double &dt, const double &diffx,
                                       double &vx) {
  vx = xPID.calculate(dt, 0, -diffx);
}
void OperatorController::GetLinearYVel(const double &dt, const double &diffy,
                                       double &vy) {
  vy = yPID.calculate(dt, 0, -diffy);
}
void OperatorController::GetLinearZVel(const double &dt, const double &diffz,
                                       double &vz) {
  vz = zPID.calculate(dt, 0, -diffz);
}

int main(int argc, char **argv) {
  auto log = CreateLogger("Main");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  log->LogToConsole(true);
  log->SetAsyncMode();

  ros::init(argc, argv, "hil_leader");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  OperatorController op(nh);
  op.Start();

  while (1) {
    ros::spinOnce();
    rate.sleep();
  }
}
