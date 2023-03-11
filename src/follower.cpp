#include <uvdar_leader_follower/follower.h>
#include <uvdar_leader_follower/FollowerConfig.h>
#include "geometry_msgs/PoseArray.h"
#include <cmath>
#include <boost/circular_buffer.hpp>

#include <mrs_lib/batch_visualizer.h>          // Circular dependency issue?
#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#define PI_RAD 3.14159

// Debugging
ros::Publisher estimate_publisher;    // PoseArray with required debugged Poses/States
ros::Publisher covariance_publisher;  // Visualization Marker rviz
geometry_msgs::PoseArray leader_estimate_pose_array_debug;
std_msgs::Header header_debug;

bool is_initialized     = false;
bool got_odometry       = false;
bool got_tracker_output = false;
bool got_uvdar          = false;

Eigen::Vector3d follower_position_odometry;
Eigen::Vector3d follower_linear_velocity_odometry;
double          follower_heading_odometry;
double          follower_heading_rate_odometry;

Eigen::Vector3d follower_position_tracker;
Eigen::Vector3d follower_linear_velocity_tracker;
Eigen::Vector3d follower_linear_acceleration_tracker;
double          follower_heading_tracker;
double          follower_heading_rate_tracker;
double          follower_heading_acceleration_tracker;

Eigen::Vector3d leader_position;
ros::Time       last_leader_contact;
bool            is_leader_moving = false;
int             number_of_moving_detections = 0;

// dynamically reconfigurable
Eigen::Vector3d position_offset          = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset           = 0.0;
double          uvdar_msg_interval       = 0.1;
bool            use_estimator            = true;    // Use LKF for predicting leader position and velocity
bool            use_speed_tracker        = false;
bool            use_trajectory_reference = true;
// Custom parameters, not dynamically reconfigurable
double safe_distance                = 3.5;              // Safe distance for target pose calculation
double observation_distance         = 3.0;              // Offset from tangentially safe distance for better observation -> to implement
double velocity_estimator_threshold = 0.05;             // Threshold to toggle 'gaze control' based on velocity estimate
double camera_heading_offset        = 32/180 * PI_RAD;  // Orientation of camera optical axis in UAV body frame
bool use_perception_aware           = false;            // Flag for activating perception aware planning mechanism
int path_window_size                = 5;                // Window size of poses for velocity estimation from raw points data
double estimator_weight             = 0.3;              // Relative weight between two velocity estimates
double traj_prediction_horizon     = 0.0;              // Prediction horizon for trajectory generation

VelocityEstimator estimator;
Eigen::Vector3d leader_predicted_position;
Eigen::Vector3d leader_corrected_position;
Eigen::Vector3d leader_predicted_velocity;
Eigen::Vector3d leader_corrected_velocity;
Eigen::Vector3d average_velocity;                         // 'Pseudo' velocity observation

Eigen::Matrix<double, 6, 6> estimate_covariance_matrix;   // Covariance of state estimate

// Trajectory planning
ros::Time           last_planned_time;                        // Last timestamp when trajectory was planned/replanned
ros::Duration       replanning_interval; // Interval after which replanning is to be done (define with respect to prediction horizon)
double              replanning_interval_s = 0.0;
bool                planning_first_trajectory = true;
ReferenceTrajectory previous_trajectory;
// Replanning updated variables
double              leader_horizon_position_x;
double              leader_horizon_position_y;
VectorWithHeading   leader_pose_with_heading;

double leader_velocity_x;
double leader_velocity_y;
double leader_velocity_heading;

//// Debug pose with covariance stamped
geometry_msgs::PoseWithCovarianceStamped pose_estimate_with_cov;

// Constructor for FollowerController
FollowerController::FollowerController(){
  this->pose_stamped_buffer = boost::circular_buffer<VectorWithHeading>(10);
  // ROS_INFO("Control action interval initialized: %f", control_action_interval);
}

double FollowerController::getControlActionInterval() {
    // ROS_INFO("Returning control_action_interval %f", control_action_interval);
    return control_action_interval;
  }

void FollowerController::initializePoseBuffer(int window_size){
  try{
    this->pose_stamped_buffer = boost::circular_buffer<VectorWithHeading>(10);
  }
  catch(const std::exception &exc){
    ROS_INFO("Caught exception");
    std::cerr << exc.what();
  }  
}

void FollowerController::updatePoseBuffer(const Eigen::Vector3d& latest_pose, ros::Time timestamp){
  
  double timestamp_sec = timestamp.sec + timestamp.nsec * 10e-9; // For average velocity calculation
  VectorWithHeading pose;
  pose.vector = latest_pose;
  pose.stamp = timestamp;
  // Unused entries in struct
  pose.heading_2d = 0.0;
  pose.value = 1.0;
  
  // pose_stamped_buffer data type is VectorWithHeading
  auto vector = this->pose_stamped_buffer[-1].vector;
  if(!(abs(vector.x() - pose.vector.x()) < std::numeric_limits<double>::epsilon() && abs(vector.y() - pose.vector.y()) < std::numeric_limits<double>::epsilon())){
    // Checking for repeat pose
    this->pose_stamped_buffer.push_back(pose);
    this->computeVelocityEstimate();
  }
  // else{
    // ROS_INFO("Skipped adding repeated pose.");
  // }
}

// Internal method to update velocity heading estimate
void FollowerController::computeVelocityEstimate(){

  int size = this->pose_stamped_buffer.size();
  if(!this->pose_stamped_buffer.empty() && size > 2){

    double average_x = 0.0, average_y = 0.0, average_z = 0.0;
    for(int i = 1; i < size; i++){
      // Calculating timestep difference between consecutive poses
      double time_step = (this->pose_stamped_buffer[i].stamp.sec - this->pose_stamped_buffer[i - 1].stamp.sec) + (this->pose_stamped_buffer[i].stamp.nsec - this->pose_stamped_buffer[i - 1].stamp.nsec) * 1e-9;
      // Calculating average velocities
      average_x += (this->pose_stamped_buffer[i].vector.x() - this->pose_stamped_buffer[i - 1].vector.x()) / time_step;
      average_y += (this->pose_stamped_buffer[i].vector.y() - this->pose_stamped_buffer[i - 1].vector.y()) / time_step;
      average_z += (this->pose_stamped_buffer[i].vector.z() - this->pose_stamped_buffer[i - 1].vector.z()) / time_step;
      }

      average_x /= size;
      average_y /= size;
      average_z /= size;
      // ROS_INFO("x: %.4f y: %.4f z: %.4f", average_x, average_y, average_z);

      this->current_velocity.vector     = Eigen::Vector3d(average_x, average_y, average_z);
      this->current_velocity.heading_2d = std::atan2(average_y, average_x);
      this->current_velocity.value      = current_velocity.vector.norm();
      this->current_velocity.stamp      = ros::Time::now();
  } else {
    // Only one pose in buffer
    this->current_velocity.vector     = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->current_velocity.heading_2d = 0.0;
    this->current_velocity.value      = 0.0;
    this->current_velocity.stamp      = ros::Time::now();
  }
}

double FollowerController::getPAHeading(const VectorWithHeading& leader_pose_with_heading, const Eigen::Vector3d& follower_position, double camera_heading_offset){
  
  double leader_velocity_heading = leader_pose_with_heading.heading_2d;
  double interval_10_deg = PI_RAD / 18;
  // ROS_INFO("Original heading angle: %f", leader_velocity_heading);
  for(int i = 0; i < 18; i++){
    if((i * interval_10_deg < abs(leader_velocity_heading)) and (abs(leader_velocity_heading) < (i+1) * interval_10_deg)){
      if(leader_velocity_heading > 0){
        leader_velocity_heading = i * interval_10_deg;
      } else {
        leader_velocity_heading = - i * interval_10_deg;
      }
      break;
    } else {
      continue;
    }
  }
  // ROS_INFO("Rounded heading angle: %f", leader_velocity_heading);
  int velocity_rotation_sign = 1;
  
  Eigen::Vector2d rotated_velocity(-std::sin(leader_velocity_heading), std::cos(leader_velocity_heading));
  Eigen::Vector2d relative_pose((leader_pose_with_heading.vector.x() - follower_position.x()), (leader_pose_with_heading.vector.y() - follower_position.y()));
  if(relative_pose.dot(rotated_velocity) > 0.0){
    velocity_rotation_sign = 1;
  } else{
    velocity_rotation_sign = -1;
  }

  return (leader_velocity_heading + (velocity_rotation_sign * PI_RAD/2) + camera_heading_offset);

}

double FollowerController::evaluatePolynomial(double coefficients[], int degree, double tau){
  
  assert(sizeof(coefficients)/sizeof(coefficients[0]) == degree);
  double power = 1;   // data type matters :)
  double result = 0.0;
  // Efficient evaluation of polynomial value
  for(int i = 0; i <= degree; i++){
    result += coefficients[i] * power;
    power *= tau;
  }
  return result;
}

Eigen::Matrix<double, 12, 1> FollowerController::getMinJerkTrajectoryCoefficients(double t0, double t1, double t2, double x0, double x1, double x2, double v0, double v2, double a0, double a2){
  assert(t2 > t1 && t1 > t0);
  // Solving for Ax = B
  Eigen::Matrix<double, 12, 12> A;
  Eigen::Matrix<double, 12, 1> b;
  Eigen::Matrix<double, 12, 1> x;

  b << x0, x1, x1, x2, v0, a0, v2, a2, 0, 0, 0, 0;
  A << pow(t0, 5), pow(t0, 4), pow(t0, 3), pow(t0, 2), pow(t0, 1), pow(t0, 0), 0, 0, 0, 0, 0, 0,
        pow(t1, 5), pow(t1, 4), pow(t1, 3), pow(t1, 2), pow(t1, 1), pow(t1, 0), 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, pow(t1, 5), pow(t1, 4), pow(t1, 3), pow(t1, 2), pow(t1, 1), pow(t1, 0),
        0, 0, 0, 0, 0, 0, pow(t2, 5), pow(t2, 4), pow(t2, 3), pow(t2, 2), pow(t2, 1), pow(t2, 0),
        5 * pow(t0, 4), 4 * pow(t0, 3), 3 * pow(t0, 2), 2 * pow(t0, 1), pow(t0, 0), 0, 0, 0, 0, 0, 0, 0,
        20 * pow(t0, 3), 12 * pow(t0, 2), 6 * pow(t0, 1), 2 * pow(t0, 0), 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,  5 * pow(t2, 4),  4 * pow(t2, 3), 3 * pow(t2, 2), 2 * pow(t2, 1), pow(t2, 0), 0,
        0, 0, 0, 0, 0, 0, 20 * pow(t2, 3), 12 * pow(t2, 2), 6 * pow(t2, 1), 2 * pow(t2, 0), 0,          0,
        5 * pow(t1, 4),   4 * pow(t1, 3),  3 * pow(t1, 2),   2 * pow(t1, 1),   pow(t1, 0),  0, 
        -5 * pow(t1, 4), -4 * pow(t1, 3), -3 * pow(t1, 2),  -2 * pow(t1, 1),  -pow(t1, 0),  0,
        20 * pow(t0, 3),     12 * pow(t0, 2),   6 * pow(t0, 1),     2 * pow(t0, 0),     0,     0, 
        -20 * pow(t0, 3),    -12 * pow(t0, 2), -6 * pow(t0, 1),   -2 * pow(t0, 0),    0,     0,
        60 * pow(t0, 2),   24 * pow(t0, 1),   6 * pow(t0, 0),   0,  0,  0, 
        -60 * pow(t0, 2),  -24 * pow(t0, 1),  -6 * pow(t0, 0),   0,  0,  0,
        120 * pow(t0, 1),   24 * pow(t0, 0),  0,  0,  0,  0, 
        -120 * pow(t0, 1),  -24 * pow(t0, 0),  0,  0,  0,  0;
  try{
    x = A.colPivHouseholderQr().solve(b); // Rank Revealing QR decomposition of matrix with column pivoting
  }
  catch(std::exception &e){
    ROS_INFO("Exception caught while calculating minimum jerk trajectory. %s. Returning empty coefficients.", e.what());
  }
  return x;
  
}

void FollowerController::publishDebug(double pa_x, double pa_y, double pa_z, double pa_heading, double pa_obs_x, double pa_obs_y, double leader_velocity_heading){

  
  // ROS_INFO("Published estimate.");
  
}

/* initialize //{ */
uvdar_leader_follower::FollowerConfig FollowerController::initialize(mrs_lib::ParamLoader& param_loader) {

  ROS_INFO("[Follower]: Waiting for odometry and uvdar");   // Waits for odometry and UVDAR Pose Filtered (by an independent intrinsic KF)
  while (ros::ok()) {
    if (got_odometry && got_uvdar) {
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  // Initializing debugging visualiser: very ugly way
  int argc = 0;
  char** argv;

  ros::init(argc, argv, "follower_debugger");
  ros::NodeHandle nh    = ros::NodeHandle("~");
  // mrs_lib::BatchVisualizer batch_visualizer = mrs_lib::BatchVisualizer(nh, "follower_debug", "uav2/gps_origin");
  estimate_publisher    = nh.advertise<geometry_msgs::PoseArray>("/leader_estimate/pose_array/debug", 2);
  covariance_publisher  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/estimate_with_covariance/debug", 2);
  ROS_INFO("Initialized Debugger Node");

  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 3, 3> R;
  param_loader.loadMatrixStatic("Q", Q);
  param_loader.loadMatrixStatic("R", R);
  param_loader.loadParam("control_action_interval", control_action_interval);

  this->getControlActionInterval();

  param_loader.loadParam("desired_offset/x", position_offset.x());
  param_loader.loadParam("desired_offset/y", position_offset.y());
  param_loader.loadParam("desired_offset/z", position_offset.z());
  param_loader.loadParam("heading_offset", heading_offset);
  // Custom parameters for perception based planning
  param_loader.loadParam("safe_distance", safe_distance);
  param_loader.loadParam("observation_distance", observation_distance);
  param_loader.loadParam("velocity_estimator_threshold", velocity_estimator_threshold);
  param_loader.loadParam("camera_heading_offset", camera_heading_offset);
  param_loader.loadParam("use_perception_aware", use_perception_aware);
  param_loader.loadParam("use_trajectory_reference", use_trajectory_reference);
  param_loader.loadParam("use_speed_tracker", use_speed_tracker);
  param_loader.loadParam("path_window_size", path_window_size);
  param_loader.loadParam("estimator_weight", estimator_weight);
  param_loader.loadParam("traj_prediction_horizon", traj_prediction_horizon);
  param_loader.loadParam("replanning_interval_s", replanning_interval_s);
  replanning_interval = ros::Duration(replanning_interval_s);

  //// initialize the dynamic reconfigurables with values from YAML file and values set above
  uvdar_leader_follower::FollowerConfig config;
  config.desired_offset_x         = position_offset.x();
  config.desired_offset_y         = position_offset.y();
  config.desired_offset_z         = position_offset.z();
  config.heading_offset           = heading_offset;
  config.filter_data              = use_estimator;
  config.use_trajectory_reference = use_trajectory_reference;
  config.use_speed_tracker        = use_speed_tracker;

  VelocityEstimator::kalman3D::x_t initial_states;

  // set initial state of estimator as follows: leader position: (current follower pos - desired offset), leader velocity: (0,0,0)
  leader_predicted_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  initial_states << follower_position_odometry.x() - position_offset.x(), follower_position_odometry.y() - position_offset.y(),
      follower_position_odometry.z() - position_offset.z(), 0, 0, 0;
  estimator = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);

  is_initialized = true;
  return config;
}
//}

/* dynamicReconfigureCallback //{ */
void FollowerController::dynamicReconfigureCallback(uvdar_leader_follower::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  position_offset               = Eigen::Vector3d(config.desired_offset_x, config.desired_offset_y, config.desired_offset_z);
  heading_offset                = config.heading_offset;
  use_speed_tracker             = config.use_speed_tracker;
  use_trajectory_reference      = config.use_trajectory_reference;
  // Custom params
  // safe_distance                 = config.safe_distance; 
  // observation_distance          = config.observation_distance;
  // velocity_estimator_threshold  = config.velocity_estimator_threshold;
  // camera_heading_offset         = config.camera_heading_offset;
  // use_perception_aware          = config.use_perception_aware;

  if (!use_estimator && config.filter_data) {
    ROS_INFO("[%s]: Estimator started", ros::this_node::getName().c_str());
  }
  use_estimator = config.filter_data;
}
//}

/* receiveOdometry //{ */
void FollowerController::receiveOdometry(const nav_msgs::Odometry& odometry_msg) {

  // Global (replace!) header for debug visualiser
  header_debug = odometry_msg.header;

  follower_position_odometry.x() = odometry_msg.pose.pose.position.x;
  follower_position_odometry.y() = odometry_msg.pose.pose.position.y;
  follower_position_odometry.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  follower_heading_odometry = ac.getHeading();

  follower_linear_velocity_odometry.x() = odometry_msg.twist.twist.linear.x;
  follower_linear_velocity_odometry.y() = odometry_msg.twist.twist.linear.y;
  follower_linear_velocity_odometry.z() = odometry_msg.twist.twist.linear.z;

  follower_heading_rate_odometry =
      ac.getHeadingRate(Eigen::Vector3d(odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z));

  got_odometry = true;
}
//}

/* receiveTrackerOutput //{ */
void FollowerController::receiveTrackerOutput(const mrs_msgs::PositionCommand& position_cmd) {

  follower_position_tracker.x() = position_cmd.position.x;
  follower_position_tracker.y() = position_cmd.position.y;
  follower_position_tracker.z() = position_cmd.position.z;

  follower_heading_tracker = position_cmd.heading;

  follower_linear_velocity_tracker.x() = position_cmd.velocity.x;
  follower_linear_velocity_tracker.y() = position_cmd.velocity.y;
  follower_linear_velocity_tracker.z() = position_cmd.velocity.z;

  follower_linear_acceleration_tracker.x() = position_cmd.acceleration.x;
  follower_linear_acceleration_tracker.y() = position_cmd.acceleration.y;
  follower_linear_acceleration_tracker.z() = position_cmd.acceleration.z;

  follower_heading_rate_tracker = position_cmd.heading_rate;
  follower_heading_acceleration_tracker = position_cmd.heading_acceleration;

  got_tracker_output = true;
}
//}

/* receiveUvdar //{ */
void FollowerController::receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg) {

  Eigen::Vector3d leader_new_position;
  Eigen::Matrix<double, 3, 3> leader_new_position_covariance;

  // Populating covariance matrix from UVDAR Kalman
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      int index = 6*i + j;
      leader_new_position_covariance << 5 * uvdar_msg.pose.covariance[index]; // Increasing covariance
      // ROS_INFO("Populated %f at index %d", uvdar_msg.pose.covariance[index], index);
    }
    
  }

  leader_new_position.x() = uvdar_msg.pose.pose.position.x;
  leader_new_position.y() = uvdar_msg.pose.pose.position.y;
  leader_new_position.z() = uvdar_msg.pose.pose.position.z;

  int difference_sec  = uvdar_msg.header.stamp.sec - last_leader_contact.sec;
  int difference_nsec = (uvdar_msg.header.stamp.nsec - last_leader_contact.nsec);
  double net_difference = difference_sec + difference_nsec * 1e-9;
  // ROS_INFO("%f", net_difference);

  last_leader_contact = uvdar_msg.header.stamp;
  got_uvdar           = true;

  leader_position = leader_new_position;

  if (use_estimator && is_initialized) {
    // Adding UVDAR position | average velocity as an observation to the LKF
    // auto leader_correction = estimator.fuse(leader_new_position, leader_new_position_covariance);
    auto leader_correction = estimator.fuse(leader_new_position);
    // auto leader_correction = estimator.fuse(leader_new_position, average_velocity);
    leader_corrected_position = Eigen::Vector3d(leader_correction[0], leader_correction[1], leader_correction[2]);
    leader_corrected_velocity = Eigen::Vector3d(leader_correction[3], leader_correction[4], leader_correction[5]);
    
    // Updating path buffer with raw readings to produce pseudo measurement of velocity
    // path_estimator.update_pose_buffer(leader_new_position, last_leader_contact);
    // ROS_INFO("Calling update pose buffer");
    updatePoseBuffer(leader_new_position, last_leader_contact);
    // ROS_INFO("Calling update pose buffer complete.");
  }
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }

  // average_velocity                = path_estimator.get_average_velocity();
  VectorWithHeading est_velocity    = this->getVelocityEstimate();
  Eigen::Vector3d average_velocity  = est_velocity.vector;
  
  // Estimator weight is a tunable parameter for weighing velocity direction
  // double leader_velocity_x          = estimator_weight * average_velocity.x() + (1 - estimator_weight) * leader_corrected_velocity.x();  1
  // double leader_velocity_y          = estimator_weight * average_velocity.y() + (1 - estimator_weight) * leader_corrected_velocity.y();  1
  double leader_velocity_x          = estimator_weight * average_velocity.x() + (1 - estimator_weight) * leader_predicted_velocity.x();
  double leader_velocity_y          = estimator_weight * average_velocity.y() + (1 - estimator_weight) * leader_predicted_velocity.y();
  double leader_velocity_heading    = std::atan2(leader_velocity_y, leader_velocity_x);
  // double leader_velocity_heading    = this->current_velocity.heading_2d;
  // double leader_velocity_norm       = sqrt(pow(leader_velocity_x, 2) + pow(leader_velocity_y, 2));
  double leader_velocity_norm       = sqrt(pow(estimator_weight * average_velocity.x() + (1 - estimator_weight) * leader_corrected_velocity.x(), 2) + pow(estimator_weight * average_velocity.y() + (1 - estimator_weight) * leader_corrected_velocity.y(), 2));
  // ROS_INFO("Leader velocity norm: %f", leader_velocity_norm);
  estimate_covariance_matrix        = estimator.getCovariance();
  
  if (use_estimator) {
    
    float avg_velocity_norm = est_velocity.value;
    
    // Perception Aware (PA) Heading calculation 
    VectorWithHeading leader_pose_with_heading;
    // leader_pose_with_heading.vector = leader_corrected_position; 1
    leader_pose_with_heading.vector     = leader_predicted_position;
    leader_pose_with_heading.heading_2d = leader_velocity_heading;

    // Computing perception aware target pose
    // double pa_x = leader_corrected_position.x() + (safe_distance * std::cos(leader_velocity_heading - PI_RAD/2));  1
    // double pa_y = leader_corrected_position.y() + (safe_distance * std::sin(leader_velocity_heading - PI_RAD/2));  1
    double pa_x = leader_predicted_position.x() + (safe_distance * std::cos(leader_velocity_heading - PI_RAD/2));
    double pa_y = leader_predicted_position.y() + (safe_distance * std::sin(leader_velocity_heading - PI_RAD/2));
    double pa_z = 3.0;
    double pa_heading = this->getPAHeading(leader_pose_with_heading, follower_position_odometry, camera_heading_offset);
    
    // Target pose with observation_distance offset along the estimated velocity direction
    double pa_obs_x = pa_x + observation_distance * std::cos(leader_velocity_heading);
    double pa_obs_y = pa_y + observation_distance * std::sin(leader_velocity_heading);

    if(use_perception_aware){
      if(leader_velocity_norm > velocity_estimator_threshold){
      /*
      Use velocity heading estimate only if leader is moving sufficiently (i.e. faster than velocity_estimator_threshold)
      Otherwise velocity direction is susceptible to noise */
        // point.position.x()  = leader_predicted_position.x() + position_offset.x();
        // point.position.y()  = leader_predicted_position.y() + position_offset.y();
        ROS_INFO("Using PA ReferencePoint.");
        point.position.x()  = pa_obs_x;
        point.position.y()  = pa_obs_y;
        point.position.z()  = 3.0;
        point.heading       = pa_heading;
      } else{
        ROS_WARN("Using constant offset ReferencePoint because heading is unreliable.");
      // Only heading is PA 
        point.position.x() = leader_predicted_position.x() + position_offset.x();
        point.position.y() = leader_predicted_position.y() + position_offset.y();
        point.position.z() = 3.0;
        point.heading      = heading_offset;
      }

    } else{
      // Using constant offset (default) | no Perception Aware
      ROS_WARN("Using constant offset ReferencePoint because estimator is off.");
      point.heading       = heading_offset;
      point.position.x()  = leader_predicted_position.x() + position_offset.x();
      point.position.y()  = leader_predicted_position.y() + position_offset.y();
      point.position.z()  = 3.0;
    }
    
    // double pa_x, double pa_y, double pa_z, double pa_heading, double pa_obs_x, double pa_obs_y, double leader_velocity_heading
    geometry_msgs::PoseArray leader_estimate_pose_array_debug;
    // Leader pose estimate visualiser declarations and assignment
    geometry_msgs::Pose leader_velocity_pose;
    leader_velocity_pose.position.x = leader_predicted_position.x();
    leader_velocity_pose.position.y = leader_predicted_position.y();
    leader_velocity_pose.position.z = leader_predicted_position.z();

    mrs_lib::EulerAttitude attitude(0.0, 0.0, leader_velocity_heading);
    mrs_lib::AttitudeConverter ac(attitude);
    // Using operator overloading provided in mrs_lib (https://ctu-mrs.github.io/mrs_lib/classmrs__lib_1_1AttitudeConverter.html)
    geometry_msgs::Quaternion orientation = ac;       // Typecasts to geometry_msgs/Orientation
    leader_velocity_pose.orientation = orientation;
    leader_estimate_pose_array_debug.poses.push_back(leader_velocity_pose);

    // With covariance
    pose_estimate_with_cov.header = header_debug;
    pose_estimate_with_cov.pose.pose = leader_velocity_pose;
    pose_estimate_with_cov.pose.covariance[0] = estimate_covariance_matrix(0, 0);
    pose_estimate_with_cov.pose.covariance[7] = estimate_covariance_matrix(1, 1);
    pose_estimate_with_cov.pose.covariance[14] = estimate_covariance_matrix(2, 2);
    covariance_publisher.publish(pose_estimate_with_cov);

    // Perception aware pose without observation_distance
    geometry_msgs::Pose pa_pose;
    pa_pose.position.x = pa_x;
    pa_pose.position.y = pa_y;
    pa_pose.position.z = pa_z;
    pa_pose.orientation = ac.setHeading(pa_heading);
    // leader_estimate_pose_array_debug.poses.push_back(pa_pose);                            // Commented out

    // Perception aware pose with observaion_distance
    pa_pose.position.x = pa_obs_x;
    pa_pose.position.y = pa_obs_y;
    pa_pose.position.z = pa_z;
    pa_pose.orientation = ac.setHeading(pa_heading);
    leader_estimate_pose_array_debug.poses.push_back(pa_pose);

    // Velocity estimate based on average velocities
    leader_velocity_pose.orientation = ac.setHeading(this->current_velocity.heading_2d);
    leader_estimate_pose_array_debug.poses.push_back(leader_velocity_pose);            // Commented out

    // Debug publish call
    // Now publishing 4 estimates: leader_pose_with_velocity | follower_target_without_obs | follower_target_with_obs | estimated average vel from path
    leader_estimate_pose_array_debug.header = header_debug;
    leader_estimate_pose_array_debug.header.stamp = ros::Time::now();
    estimate_publisher.publish(leader_estimate_pose_array_debug);
    // this->publishDebug(pa_x, pa_y, pa_z, pa_heading, pa_obs_x, pa_obs_y, leader_velocity_heading);
    
  } else {
    // If not using estimator for leader velocity, use constant offset
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = 3.0;
  }
  
  // Using reference point
  point.use_for_control = true;

  previous_point = point;
  return point;
}
//}

/* createReferenceTrajectory //{ */
ReferenceTrajectory FollowerController::createReferenceTrajectory() {
  ReferenceTrajectory trajectory;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    trajectory.positions.push_back(Eigen::Vector3d::Zero());
    trajectory.headings.push_back(0.0);
    trajectory.sampling_time   = 0.0;
    trajectory.use_for_control = false;
    return trajectory;
  }

  // Example - start trajectory at current UAV position and move in the predicted direction of leader motion
  // No subsampling, only two points are created in this example
  Eigen::Vector3d point_1;
  double          heading_1;

  Eigen::Vector3d point_2;
  double          heading_2;

  trajectory.use_for_control = false;
  if (use_trajectory_reference) {
    if (use_estimator) {
      
      // Check if leader velocity estimate is reliable
      VectorWithHeading est_velocity    = this->getVelocityEstimate();
      Eigen::Vector3d average_velocity  = est_velocity.vector;
      double leader_velocity_norm       = sqrt(pow(estimator_weight * average_velocity.x() + (1 - estimator_weight) * leader_corrected_velocity.x(), 2) + pow(estimator_weight * average_velocity.y() + (1 - estimator_weight) * leader_corrected_velocity.y(), 2));

      if(leader_velocity_norm > velocity_estimator_threshold){
        // Perception aware pose is valid
        leader_velocity_x = estimator_weight * average_velocity.x() + (1 - estimator_weight) * leader_predicted_velocity.x();
        leader_velocity_y = estimator_weight * average_velocity.y() + (1 - estimator_weight) * leader_predicted_velocity.y();
        leader_velocity_heading      = std::atan2(leader_velocity_y, leader_velocity_x);

        leader_pose_with_heading.vector     = leader_corrected_position;
        leader_pose_with_heading.heading_2d = leader_velocity_heading;
        leader_horizon_position_x    = leader_predicted_position.x() + (leader_predicted_velocity.x() * traj_prediction_horizon);
        leader_horizon_position_y    = leader_predicted_position.y() + (leader_predicted_velocity.y() * traj_prediction_horizon);

        double pa_x = leader_horizon_position_x + (safe_distance * std::cos(leader_velocity_heading - PI_RAD/2));
        double pa_y = leader_horizon_position_y + (safe_distance * std::sin(leader_velocity_heading - PI_RAD/2));
        double pa_z = 3.0;
        double pa_heading = this->getPAHeading(leader_pose_with_heading, follower_position_odometry, camera_heading_offset);
        
        // Target pose with observation_distance offset along the estimated velocity direction
        double pa_obs_x = pa_x + observation_distance * std::cos(leader_velocity_heading);
        double pa_obs_y = pa_y + observation_distance * std::sin(leader_velocity_heading);


        point_1     = follower_position_tracker;
        point_1.z() = 3.0;
        heading_1   = follower_heading_tracker;
        trajectory.positions.push_back(point_1);
        trajectory.headings.push_back(heading_1);

        // Replacing with sampled trajectory

        double coeff_x1[6];
        // double coeff_x2[6];
        double coeff_y1[6];
        // double coeff_y2[6];
        double coeff_h1[6];
        // double coeff_h2[6];

        double D = traj_prediction_horizon;
        // X Trajectory
        double x_i      = follower_position_tracker.x();
        double vx_i     = follower_linear_velocity_tracker.x();
        double ax_i     = follower_linear_acceleration_tracker.x();
        double x_f      = pa_obs_x;
        double vx_f     = 0.0;
        double ax_f     = 0.0;
        double delta_x  = x_f - x_i;
        // Coefficients from initial conditions
        coeff_x1[0] = x_i;
        coeff_x1[1] = D * vx_i;
        coeff_x1[2] = 0.5 * pow(D, 2) * ax_i;
        // Coefficients from fina1 conditions
        coeff_x1[3] = -((3/2) * pow(D, 2)) - (6 * D * vx_i) + (10 * delta_x);
        coeff_x1[4] = +((3/2) * pow(D, 2)) + (8 * D * vx_i) - (15 * delta_x);
        coeff_x1[5] = -((1/2) * pow(D, 2)) - (3 * D * vx_i) + (06 * delta_x);

        // Y Trajectory
        double y_i      = follower_position_tracker.y();
        double vy_i     = follower_linear_velocity_tracker.y();
        double ay_i     = follower_linear_acceleration_tracker.y();
        double y_f      = pa_obs_y;
        double vy_f     = 0.0;
        double ay_f     = 0.0;
        double delta_y  = y_f - y_i;
        // Coefficients from initial conditions
        coeff_y1[0] = y_i;
        coeff_y1[1] = D * vy_i;
        coeff_y1[2] = 0.5 * pow(D, 2) * ay_i;
        // Coefficients from fina1 conditions
        coeff_y1[3] = -((3/2) * pow(D, 2)) - (6 * D * vy_i) + (10 * delta_y);
        coeff_y1[4] = +((3/2) * pow(D, 2)) + (8 * D * vy_i) - (15 * delta_y);
        coeff_y1[5] = -((1/2) * pow(D, 2)) - (3 * D * vy_i) + (06 * delta_y);
        
        // Heading Trajectory
        double h_i      = follower_heading_tracker;
        double vh_i     = follower_heading_rate_tracker;
        double ah_i     = follower_heading_acceleration_tracker;
        double h_f      = pa_heading;
        double vh_f     = 0.0;
        double ah_f     = 0.0;
        double delta_h  = h_f - h_i;
        // Coefficients from initial conditions
        coeff_h1[0] = h_i;
        coeff_h1[1] = D * vh_i;
        coeff_h1[2] = 0.5 * pow(D, 2) * ah_i;
        // Coefficients from fina1 conditions
        coeff_h1[3] = -((3/2) * pow(D, 2)) - (6 * D * vh_i) + (10 * delta_h);
        coeff_h1[4] = +((3/2) * pow(D, 2)) + (8 * D * vh_i) - (15 * delta_h);
        coeff_h1[5] = -((1/2) * pow(D, 2)) - (3 * D * vh_i) + (06 * delta_h);

        for(int t = 0; t < 10 * traj_prediction_horizon; t++){

          Eigen::Vector3d point;
          double          heading;
          double          tau = t / (10 * traj_prediction_horizon);

          point.x() = this->evaluatePolynomial(coeff_x1, 5, tau);
          point.y() = this->evaluatePolynomial(coeff_y1, 5, tau);
          point.z() = 3.0;
          heading   = this->evaluatePolynomial(coeff_h1, 5, tau);

          trajectory.positions.push_back(point);
          trajectory.headings.push_back(heading);

        }

        //// Publish debug
        geometry_msgs::PoseArray leader_estimate_pose_array_debug;
        // Leader pose estimate visualiser declarations and assignment
        geometry_msgs::Pose leader_velocity_pose;
        leader_velocity_pose.position.x = leader_predicted_position.x();
        leader_velocity_pose.position.y = leader_predicted_position.y();
        leader_velocity_pose.position.z = leader_predicted_position.z();

        mrs_lib::EulerAttitude attitude(0.0, 0.0, leader_velocity_heading);
        mrs_lib::AttitudeConverter ac(attitude);
        // Using operator overloading provided in mrs_lib (https://ctu-mrs.github.io/mrs_lib/classmrs__lib_1_1AttitudeConverter.html)
        geometry_msgs::Quaternion orientation = ac;       // Typecasts to geometry_msgs/Orientation
        leader_velocity_pose.orientation = orientation;
        leader_estimate_pose_array_debug.poses.push_back(leader_velocity_pose);

        // With covariance
        pose_estimate_with_cov.header = header_debug;
        pose_estimate_with_cov.pose.pose = leader_velocity_pose;
        pose_estimate_with_cov.pose.covariance[0] = estimate_covariance_matrix(0, 0);
        pose_estimate_with_cov.pose.covariance[7] = estimate_covariance_matrix(1, 1);
        pose_estimate_with_cov.pose.covariance[14] = estimate_covariance_matrix(2, 2);
        covariance_publisher.publish(pose_estimate_with_cov);

        // Perception aware pose without observation_distance
        geometry_msgs::Pose pa_pose;
        pa_pose.position.x = pa_x;
        pa_pose.position.y = pa_y;
        pa_pose.position.z = pa_z;
        pa_pose.orientation = ac.setHeading(pa_heading);
        // leader_estimate_pose_array_debug.poses.push_back(pa_pose);                            // Commented out

        // Perception aware pose with observaion_distance
        pa_pose.position.x = pa_obs_x;
        pa_pose.position.y = pa_obs_y;
        pa_pose.position.z = pa_z;
        pa_pose.orientation = ac.setHeading(pa_heading);
        leader_estimate_pose_array_debug.poses.push_back(pa_pose);

        // Velocity estimate based on average velocities
        leader_velocity_pose.orientation = ac.setHeading(this->current_velocity.heading_2d);
        // leader_estimate_pose_array_debug.poses.push_back(leader_velocity_pose);            // Commented out

        // Debug publish call
        // Now publishing 4 estimates: leader_pose_with_velocity | follower_target_without_obs | follower_target_with_obs | estimated average vel from path
        leader_estimate_pose_array_debug.header = header_debug;
        leader_estimate_pose_array_debug.header.stamp = ros::Time::now();
        estimate_publisher.publish(leader_estimate_pose_array_debug);

        //// Publish debug

        // point_2   = leader_predicted_position + position_offset + (leader_predicted_velocity * control_action_interval);
        // heading_2 = heading_offset;
        // point_2.x() = pa_obs_x;
        // point_2.y() = pa_obs_y;
        // point_2.z() = pa_z;
        // heading_2   = pa_heading;

        trajectory.sampling_time   = std::max(0.1, control_action_interval);  // Controller sampling time limit is 0.1s
        trajectory.use_for_control = true;
      } else {
        // Leader velocity heading is unreliable
        ROS_WARN("[%s]: Tried to plan a trajectory without reliable velocity heading", ros::this_node::getName().c_str());

      }
      
    } else {
      ROS_WARN("[%s]: Tried to plan a trajectory without leader velocity estimation", ros::this_node::getName().c_str());
    }
  }

  return trajectory;
}
//}
  

//}

/* createSpeedCommand //{ */
SpeedCommand FollowerController::createSpeedCommand() {
  SpeedCommand command;

  if (!got_odometry || !got_uvdar || !got_tracker_output) {
    command.velocity        = Eigen::Vector3d(0, 0, 0);
    command.heading         = 0;
    command.height          = 0;
    command.use_for_control = false;
  }

  if (use_estimator) {
    command.velocity = leader_predicted_velocity;
    command.height   = leader_predicted_position.z() + position_offset.z();
    command.heading  = follower_heading_odometry;
  }

  if (use_speed_tracker) {
    command.use_for_control = true;
  } else {
    command.use_for_control = false;
  }
  return command;
}
//}

/* getCurrentEstimate //{ */

// You can use this method for debugging purposes.
// It allows you to visualize the leader predictions in rviz
// It is called once per control action of the summer_schoo_supervisor

nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    auto leader_prediction          = estimator.predict(leader_predicted_velocity, control_action_interval);
    leader_predicted_position       = Eigen::Vector3d(leader_prediction[0], leader_prediction[1], leader_prediction[2]);
    leader_predicted_velocity       = Eigen::Vector3d(leader_prediction[3], leader_prediction[4], leader_prediction[5]);
    leader_est.pose.pose.position.x = leader_prediction[0];
    leader_est.pose.pose.position.y = leader_prediction[1];
    leader_est.pose.pose.position.z = leader_prediction[2];
    leader_est.twist.twist.linear.x = leader_prediction[3];
    leader_est.twist.twist.linear.y = leader_prediction[4];
    leader_est.twist.twist.linear.z = leader_prediction[5];
  } else {
    leader_est.pose.pose.position.x = leader_position.x();
    leader_est.pose.pose.position.y = leader_position.y();
    leader_est.pose.pose.position.z = leader_position.z();
  }

  return leader_est;
}

//}