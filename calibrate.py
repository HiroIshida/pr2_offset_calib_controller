import time

import rospy
from pr2_mechanism_msgs.srv import (ListControllers, ListControllersRequest,
                                    ListControllersResponse,
                                    ListControllerTypes,
                                    ListControllerTypesResponse,
                                    LoadController, LoadControllerRequest,
                                    LoadControllerResponse, SwitchController,
                                    UnloadController, UnloadControllerRequest)

CONTROLLER_TYPE = "pr2_offset_calib_controller/OffsetCalibrationController"
CONTROLLER_NAME = "pr2_offset_calib_controller"


def is_offset_calib_included() -> bool:
    sp = rospy.ServiceProxy("/pr2_controller_manager/list_controller_types", ListControllerTypes)
    resp: ListControllerTypesResponse = sp()
    rospy.loginfo(resp)
    is_included = CONTROLLER_TYPE in resp.types
    return is_included


def is_offset_calib_loaded() -> bool:
    sp = rospy.ServiceProxy("/pr2_controller_manager/list_controllers", ListControllers)
    request = ListControllersRequest()
    resp: ListControllersResponse = sp(request)
    rospy.loginfo(resp)
    return CONTROLLER_NAME in resp.controllers


def load_controller(joint_name: str, offset: float) -> bool:
    if not is_offset_calib_included():
        rospy.logerr("OffsetCalibrationController is not included in the controller manager.")
        return False
    rospy.loginfo("OffsetCalibrationController is included in the controller manager.")

    rospy.set_param(f"/{CONTROLLER_NAME}/type", CONTROLLER_TYPE)
    joint_name_without_joint = joint_name.replace("_joint", "")
    actuator_name = joint_name_without_joint + "_motor"
    rospy.set_param(f"/{CONTROLLER_NAME}/joint", joint_name)
    rospy.set_param(f"/{CONTROLLER_NAME}/actuator", actuator_name)
    rospy.set_param(f"/{CONTROLLER_NAME}/extra_offset", offset)
    rospy.loginfo(f"joint: {joint_name}, actuator: {actuator_name}, offset: {offset}")
    time.sleep(3.0)

    if is_offset_calib_loaded():
        rospy.logwarn("OffsetCalibrationController is already loaded.")
        sp = rospy.ServiceProxy("/pr2_controller_manager/unload_controller", UnloadController)
        request = UnloadControllerRequest(name=CONTROLLER_NAME)
        resp: LoadControllerResponse = sp(request)
        if resp.ok:
            rospy.loginfo("OffsetCalibrationController unloaded successfully!")
        else:
            rospy.logerr("Failed to unload OffsetCalibrationController.")
            return False
        time.sleep(1.0)

    sp = rospy.ServiceProxy("/pr2_controller_manager/load_controller", LoadController)
    request = LoadControllerRequest(name=CONTROLLER_NAME)
    resp: LoadControllerResponse = sp(request)
    if resp.ok:
        rospy.loginfo("OffsetCalibrationController loaded successfully!")
    else:
        rospy.logerr("Failed to load OffsetCalibrationController.")
    return resp.ok


def start_controller(joint_name):
    rarm = joint_name.startswith("r_")
    larm = joint_name.startswith("l_")
    if not (rarm or larm):
        rospy.logerr("Joint name must start with 'r_' or 'l_'.")
        return False

    if rarm:
        stop_list = ["r_arm_controller", "r_arm_controller_loose"]
    else:
        stop_list = ["l_arm_controller", "l_arm_controller_loose"]
    start_list = [CONTROLLER_NAME]

    sp = rospy.ServiceProxy("/pr2_controller_manager/switch_controller", SwitchController)
    resp = sp(start_controllers=start_list, stop_controllers=stop_list)
    rospy.loginfo("controller service response: {}".format(resp))
    if resp.ok:
        rospy.loginfo("OffsetCalibrationController started successfully!")
    else:
        rospy.logerr("Failed to start OffsetCalibrationController.")
        return False

    time.sleep(3.0)
    resp = sp(
        start_controllers=["r_arm_controller", "l_arm_controller"],
        stop_controllers=[
            CONTROLLER_NAME,
            "r_arm_controller_loose",
            "l_arm_controller_loose",
        ],
    )
    if resp.ok:
        rospy.loginfo("Reverted to original controllers successfully!")
    else:
        rospy.logerr("Failed to revert to original controllers.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", type=str, default="l_elbow_flex_joint")
    parser.add_argument("--offset", type=float, default=0.0, help="extra offset to be applied")
    args = parser.parse_args()

    rospy.init_node("offset_calib_controller")
    load_controller(args.joint, args.offset)
    start_controller(args.joint)
