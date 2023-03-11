#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/String.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <uvdar_leader_follower/FollowerConfig.h>
#include <uvdar_leader_follower/velocity_estimator.h>
#include <uvdar_leader_follower/message_utils.h>
#include <boost/circular_buffer.hpp>

struct VectorWithHeading
{
  Eigen::Vector3d vector;       // Position in 3D coordinate space in m
  double          heading_2d;   // Heading in radians in X-Y plane || unused in some cases
  double          value;        // Stores a value independent of the vector and heading || unused in some cases
  ros::Time       stamp;        // Timestamp of creation/updation
};

struct ReferencePoint
{
  Eigen::Vector3d position;
  double          heading;
  bool            use_for_control;
};

struct ReferenceTrajectory
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<double>          headings;
  double                       sampling_time;
  bool                         use_for_control;
};

struct SpeedCommand
{
  Eigen::Vector3d velocity;
  double          heading;
  double          height;
  bool            use_for_control;
};

class FollowerController {

public:
  
  FollowerController();
  ~FollowerController() {
  }

  // State machine states
  const int STATE_FOLLOW  = 0;
  const int STATE_TRACK   = 1;
  const int STATE_SEARCH  = 2;

  ReferencePoint      createReferencePoint();
  ReferenceTrajectory createReferenceTrajectory();
  SpeedCommand        createSpeedCommand();

  uvdar_leader_follower::FollowerConfig initialize(mrs_lib::ParamLoader& param_loader);

  void receiveOdometry(const nav_msgs::Odometry& odometry_msg);
  void receiveTrackerOutput(const mrs_msgs::PositionCommand& position_cmd);
  void receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg);
  void dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, uint32_t level);

  // Initialize internal pose buffer for sliding window of recent poses
  void initializePoseBuffer(int window_size);

  // Internally calls private method to update velocity estimate
  void updatePoseBuffer(const Eigen::Vector3d& latest_pose, ros::Time timestamp);
  
  nav_msgs::Odometry getCurrentEstimate();

  double getControlActionInterval();

  double getPAHeading(const VectorWithHeading& leader_pose_with_heading, const Eigen::Vector3d& follower_position_odom, double camera_heading_offset);

  // ReferenceTrajectory getSmoothTrajectory();

  double evaluatePolynomial(double coefficients[], int degree, double tau);

  Eigen::Matrix<double, 12, 1> getMinJerkTrajectoryCoefficients(double t0, double t1, double t2, double x0, double x1, double x2, double v0, double v1, double a0, double a1);

  void publishDebug(double pa_x, double pa_y, double pa_z, double pa_heading, double pa_obs_x, double pa_obs_y, double leader_velocity_heading);

  // Returns the current estimated velocity based on sliding window average of poses observed
  VectorWithHeading getVelocityEstimate(){
    return this->current_velocity;    // Confirm validity of operator
  }
  
  // bool isLeaderMoving();

private:
  double control_action_interval = 0.0;
  ReferencePoint previous_point;
  
  // Internal variables used for path based velocity direction estimation
  
  VectorWithHeading current_velocity;
  int window_size = 10;

  boost::circular_buffer<VectorWithHeading> pose_stamped_buffer;

  // Internal method to update velocity heading estimate
  void computeVelocityEstimate();

};
