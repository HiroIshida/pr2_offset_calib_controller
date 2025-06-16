#pragma once

#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_controllers_msgs/QueryCalibrationState.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Empty.h>

namespace controller
{

/** \brief Adds a user-specified extra offset to actuator->state_.zero_offset_
 *
 * Executes `zero_offset_ += extra_offset` once during starting() and marks
 * the joint as calibrated, then terminates. Minimal controller.
 *
 *  <param name="joint">      Target joint name (required)                       </param>
 *  <param name="actuator">   Target actuator name (required)                    </param>
 *  <param name="extra_offset">Offset to add [rad] (default 0.0)                 </param>
 */
class OffsetCalibrationController : public pr2_controller_interface::Controller
{
public:
  OffsetCalibrationController();
  ~OffsetCalibrationController() override;

  bool  init   (pr2_mechanism_model::RobotState* robot, ros::NodeHandle& n) override;
  void  starting() override;     ///< Add extra_offset_ to zero_offset_
  void  update  () override;
  bool  isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request&  req,
                     pr2_controllers_msgs::QueryCalibrationState::Response& resp);

private:
  ros::NodeHandle node_;
  pr2_mechanism_model::RobotState*      robot_;
  pr2_mechanism_model::JointState*      joint_;
  pr2_hardware_interface::Actuator*     actuator_;

  double extra_offset_;                       ///< [rad]
  ros::ServiceServer is_calibrated_srv_;      ///< /is_calibrated service
  boost::scoped_ptr< realtime_tools::RealtimePublisher<std_msgs::Empty> >
        pub_calibrated_;                      ///< /calibrated topic
  ros::Time last_publish_time_;
};

}  // namespace controller
