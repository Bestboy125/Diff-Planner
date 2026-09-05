// Offline test of the actual trajectory-server callbacks and yaw calculation.
// Rename its entry point: never initialize a ROS node, advertise or publish.
#define main unused_traj_server_entry
#include "../src/traj_server.cpp"
#undef main
#include <cstdlib>
#include <iostream>
#include <limits>

void require(bool value, const char* message)
{
  if (!value) { std::cerr << message << std::endl; std::exit(1); }
}

quadrotor_msgs::PositionCommandPtr heading(double yaw, double stamp)
{
  quadrotor_msgs::PositionCommandPtr msg(new quadrotor_msgs::PositionCommand);
  msg->yaw = yaw;
  msg->header.stamp = ros::Time(stamp);
  return msg;
}

void trajectory(float dx, float dy, float dz)
{
  traj_utils::PolyTrajPtr msg(new traj_utils::PolyTraj);
  msg->order = 5;
  msg->duration = {20.0};
  msg->coef_x = {0, 0, 0, 0, dx, 0};
  msg->coef_y = {0, 0, 0, 0, dy, 0};
  msg->coef_z = {0, 0, 0, 0, dz, 1};
  msg->start_time = ros::Time(100);
  polyTrajCallback(msg);
}

int main()
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(100));
  time_forward_ = 1.0;
  operatorYawHoldCallback(heading(0.7, 100));
  Eigen::Vector3d position(0, 0, 1);
  const double directions[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  for (const auto& direction : directions)
  {
    trajectory(direction[0], direction[1], direction[2]); // replans must retain hold
    for (double elapsed : {0.1, 0.6, 5.0, 15.0})
    {
      ros::Time::setNow(ros::Time(100 + elapsed));
      const auto output = calculate_yaw(elapsed, position, 0.01);
      require(std::abs(output.first - 0.7) < 1e-6, "translation changed heading");
      require(output.second == 0.0, "translation generated yaw rate");
    }
  }
  yawCallback(heading(-1.0, 99));
  yawCallback(heading(-1.0, 0));
  require(operator_yaw_hold_, "old/unstamped hint cleared hold");
  operatorYawHoldCallback(heading(-1.0, 99));
  operatorYawHoldCallback(heading(std::numeric_limits<double>::quiet_NaN(), 116));
  require(std::abs(operator_yaw_ - 0.7) < 1e-6, "invalid hold changed heading");
  operatorYawHoldCallback(heading(-3.1, 116));
  require(std::abs(calculate_yaw(1, position, 0.01).first + 3.1) < 1e-6, "new translation failed to recapture heading");

  ros::Time::setNow(ros::Time(117));
  const double previous = last_yaw_;
  yawCallback(heading(-2.0, 117)); // explicit turn / VLA yaw relinquishes hold
  require(!operator_yaw_hold_, "new normal yaw failed to release hold");
  const auto turning = calculate_yaw(1, position, 0.01);
  require(turning.first > previous, "explicit turn no longer works");
  require(std::abs(turning.second - (turning.first - previous) / 0.01) < 1e-6,
          "yaw_dot must contain angular velocity, not target angle");
  require(calculate_yaw(1, position, 0.0).second == 0.0, "zero dt generated invalid yaw rate");
  std::cout << "PASS: six directions, >0.5s hold, replans, stale/invalid hints, recapture, turn release, yaw rate" << std::endl;
  return 0;
}
