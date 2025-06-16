#include "offset_calib_controller/offset_calibration_controller.h"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(controller::OffsetCalibrationController,
                       pr2_controller_interface::Controller)

namespace controller {

OffsetCalibrationController::OffsetCalibrationController()
    : robot_(nullptr),
      joint_(nullptr),
      actuator_(nullptr),
      extra_offset_(0.0),
      last_publish_time_(0.0) {}

OffsetCalibrationController::~OffsetCalibrationController() = default;

bool OffsetCalibrationController::init(pr2_mechanism_model::RobotState* robot,
                                       ros::NodeHandle& n) {
  robot_ = robot;
  node_ = n;

  std::string joint_name, actuator_name;
  if (!node_.getParam("joint", joint_name) ||
      !node_.getParam("actuator", actuator_name)) {
    ROS_ERROR(
        "OffsetCalibrationController: Please specify both 'joint' and "
        "'actuator' "
        "(namespace: %s)",
        node_.getNamespace().c_str());
    return false;
  }

  joint_ = robot_->getJointState(joint_name);
  actuator_ = robot_->model_->getActuator(actuator_name);

  if (!joint_ || !actuator_) {
    ROS_ERROR(
        "OffsetCalibrationController: joint '%s' or actuator '%s' not found "
        "(namespace: %s)",
        joint_name.c_str(), actuator_name.c_str(),
        node_.getNamespace().c_str());
    return false;
  }

  node_.param("extra_offset", extra_offset_, 0.0);
  ROS_INFO(
      "OffsetCalibrationController: applying extra_offset = %.6f [rad] "
      "(joint=%s)",
      extra_offset_, joint_name.c_str());

  is_calibrated_srv_ = node_.advertiseService(
      "is_calibrated", &OffsetCalibrationController::isCalibrated, this);

  // /calibrated topic (once every 0.5s to match other controllers)
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(
      node_, "calibrated", 1));

  return true;
}

void OffsetCalibrationController::starting() {
  actuator_->state_.zero_offset_ += extra_offset_;
  joint_->calibrated_ = true;

  ROS_INFO(
      "OffsetCalibrationController: added %.6f [rad] to zero_offset_ "
      "(new value=%.6f)",
      extra_offset_, actuator_->state_.zero_offset_);
}

void OffsetCalibrationController::update() {
  // update() is called until controller_manager sends stop signal after
  // startup, so publish "calibrated" every 0.5s like other calibration
  // controllers
  if (pub_calibrated_) {
    ros::Time now = robot_->getTime();
    if (last_publish_time_ + ros::Duration(0.5) < now) {
      if (pub_calibrated_->trylock()) {
        last_publish_time_ = now;
        pub_calibrated_->unlockAndPublish();
      }
    }
  }
}

bool OffsetCalibrationController::isCalibrated(
    pr2_controllers_msgs::QueryCalibrationState::Request& /*req*/,
    pr2_controllers_msgs::QueryCalibrationState::Response& resp) {
  resp.is_calibrated = joint_->calibrated_;
  return true;
}

}  // namespace controller
