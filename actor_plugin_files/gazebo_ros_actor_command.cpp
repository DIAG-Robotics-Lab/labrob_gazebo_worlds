#include <gazebo_ros_actor_plugin/gazebo_ros_actor_command.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <functional>

#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>

using namespace gazebo;

#define _USE_MATH_DEFINES
#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////

// Plugin constructor

GazeboRosActorCommand::GazeboRosActorCommand() {
}

GazeboRosActorCommand::~GazeboRosActorCommand() {
  this->vel_queue_.clear();
  this->vel_queue_.disable();
  this->velCallbackQueueThread_.join();

  this->ros_node_->shutdown();
  delete this->ros_node_;
}

////////////////////////////////////////////////////////////////////////

// Initialize the plugin

void GazeboRosActorCommand::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Initialize default values for parameters
  this->follow_mode_ = "velocity";
  this->vel_topic_ = "/cmd_vel";
  this->animation_factor_ = 4.0;

  // Override default parameter values with values from SDF world file
  if (_sdf->HasElement("follow_mode")) {
    this->follow_mode_ = _sdf->Get<std::string>("follow_mode");
  }
  if (_sdf->HasElement("vel_topic")) {
    this->vel_topic_ = _sdf->Get<std::string>("vel_topic");
  }
  if (_sdf->HasElement("animation_factor")) {
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }
  if (_sdf->HasElement("default_rotation_roll")) {
    this->default_rotation_roll_ = _sdf->Get<double>("default_rotation_roll");
  }
  if (_sdf->HasElement("default_rotation_yaw")) {
    this->default_rotation_yaw_ = _sdf->Get<double>("default_rotation_yaw");
  }
  if (_sdf->HasElement("actor_type")) {
    this->actor_type_ = _sdf->Get<std::string>("actor_type");
  }
  

  // Check if ROS node for Gazebo has been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("actor", "A ROS node for Gazebo has not been "
    << "initialized, unable to load plugin. Load the Gazebo system plugin "
    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Set variables
  this->sdf_ = _sdf;
  this->actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world_ = this->actor_->GetWorld();
  this->Reset();
  this->follow_mode_ = "velocity";

  // Create ROS node handle
  this->ros_node_ = new ros::NodeHandle();

  // Subscribe to the velocity commands
  ros::SubscribeOptions vel_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
      vel_topic_,
      1,
      boost::bind(&GazeboRosActorCommand::VelCallback, this, _1),
      ros::VoidPtr(),
      &vel_queue_);
  this->vel_sub_ = ros_node_->subscribe(vel_so);

  // Create a thread for the velocity callback queue
  this->velCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosActorCommand::VelQueueThread, this));

  // Connect the OnUpdate function to the WorldUpdateBegin event.
  this->connections_.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosActorCommand::OnUpdate, this, std::placeholders::_1)));
}

////////////////////////////////////////////////////////////////////////////////////////

void GazeboRosActorCommand::Reset() {
  // Reset last update time and target pose index
  this->last_update_ = 0;
  this->idx_ = 0;
  // Initialize target poses vector with origin
  this->target_poses_.push_back(ignition::math::Vector3d(0.0, 0.0, 0.0));
  // Set target pose to the current pose
  this->target_pose_ = this->target_poses_.at(this->idx_);

  // Check if the walking animation exists in the actor's skeleton animations
  auto skelAnims = this->actor_->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WALKING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;

    // Set the actor's trajectory to the custom trajectory
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
  }
}

void GazeboRosActorCommand::VelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  ignition::math::Vector3d vel_cmd;
  vel_cmd.X() = msg->linear.x;
  vel_cmd.Y() = msg->linear.y;
  vel_cmd.Z() = msg->angular.z;
  this->cmd_queue_.push(vel_cmd);
}

/////////////////////////////////////////////////

void GazeboRosActorCommand::OnUpdate(const common::UpdateInfo &_info) {
  // Time delta
  double dt = (_info.simTime - this->last_update_).Double();
  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
if (this->follow_mode_ == "velocity" and this->actor_type_ == "omnidirectional") {
  if (!this->cmd_queue_.empty()) {
    this->target_vel_.Pos().X() = this->cmd_queue_.front().X();
    this->target_vel_.Pos().Y() = this->cmd_queue_.front().Y();
    this->cmd_queue_.pop();
  }

  // Velocità nel frame mondo
  double v_x_world = this->target_vel_.Pos().X();
  double v_y_world = this->target_vel_.Pos().Y();

  // Aggiorna orientamento per "guardare" nella direzione di movimento
  if (std::abs(v_x_world) > 1e-6 || std::abs(v_y_world) > 1e-6) {
    double des_orientation = atan2(v_y_world, v_x_world);
    pose.Rot() = ignition::math::Quaterniond(default_rotation_roll_, 0,
                                             des_orientation + default_rotation_yaw_);
  } else {
    pose.Rot() = ignition::math::Quaterniond(default_rotation_roll_, 0,
                                             rpy.Z());
  }

  // Aggiorna posizione direttamente usando la velocità nel mondo
  pose.Pos().X() += v_x_world * dt;
  pose.Pos().Y() += v_y_world * dt;
}





  else if (this->follow_mode_ == "velocity" and this->actor_type_ == "unicycle") {
    if (!this->cmd_queue_.empty()) {
      this->target_vel_.Pos().X() = this->cmd_queue_.front().X();
      this->target_vel_.Rot() = ignition::math::Quaterniond(0, 0, this->cmd_queue_.front().Z());
      this->cmd_queue_.pop();
    }

    pose.Pos().X() += this->target_vel_.Pos().X() *
                      cos(pose.Rot().Euler().Z() - default_rotation_yaw_) * dt;
    pose.Pos().Y() += this->target_vel_.Pos().X() *
                      sin(pose.Rot().Euler().Z() - default_rotation_yaw_) * dt;

    double delta_yaw = this->target_vel_.Rot().Euler().Z() * dt;
    
    pose.Rot() = ignition::math::Quaterniond(default_rotation_roll_, 0, rpy.Z() + delta_yaw);
  }

  else  {
    std::cout << "Invalid follow mode or actor type" << std::endl;
  }



  // Distance traveled is used to coordinate motion with the walking animation
  auto displacement = pose.Pos() - this->actor_->WorldPose().Pos();
  double distanceTraveled = displacement.Length();

  this->actor_->SetWorldPose(pose, false, false);
  this->actor_->SetScriptTime(
  this->actor_->ScriptTime() + (distanceTraveled * this->animation_factor_));
  this->last_update_ = _info.simTime;
}

void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;

  // Set next target
  this->target_pose_ = this->target_poses_.at(this->idx_);
}

void GazeboRosActorCommand::VelQueueThread() {
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->vel_queue_.callAvailable(ros::WallDuration(timeout));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosActorCommand)