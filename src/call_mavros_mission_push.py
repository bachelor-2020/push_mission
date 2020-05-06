#! /usr/bin/env python
from __future__ import print_function
import rospy
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint


def mission_push_client(start_index, waypoints):
    rospy.wait_for_service("mavros/mission/push")

    try:
        mission_service = rospy.ServiceProxy("mavros/mission/push", WaypointPush)
        mission_response = mission_service(start_index, waypoints)
        return mission_response.success, mission_response.wp_transfered

    except rospy.ServiceException as e:
        print("Service call failed: ", e)
        pass


if __name__ == "__main__":

    # # Waypoint.msg
    #
    # ROS representation of MAVLink MISSION_ITEM
    # See mavlink documentation

    # # see enum MAV_FRAME
    # uint8 frame
    # uint8 FRAME_GLOBAL = 0
    # uint8 FRAME_LOCAL_NED = 1
    # uint8 FRAME_MISSION = 2
    # uint8 FRAME_GLOBAL_REL_ALT = 3
    # uint8 FRAME_LOCAL_ENU = 4

    # # see enum MAV_CMD
    # uint16 command
    # uint16 NAV_WAYPOINT = 16
    # uint16 NAV_LOITER_UNLIM = 17
    # uint16 NAV_LOITER_TURNS = 18
    # uint16 NAV_LOITER_TIME = 19
    # uint16 NAV_RETURN_TO_LAUNCH = 20
    # uint16 NAV_LAND = 21
    # uint16 NAV_TAKEOFF = 22

    # bool is_current
    # bool autocontinue

    # # meaning of this params described in enum MAV_CMD
    # float32 param1
    # float32 param2
    # float32 param3
    # float32 param4
    # float64 x_lat
    # float64 y_long
    # float64 z_alt

    wp_0 = Waypoint(
        frame=3, command=22, is_current=True, autocontinue=True, z_alt=10
    )  # Takeoff, 10 meters, make current wp
    wp_1 = Waypoint(
        frame=3,
        command=16,
        is_current=False,
        autocontinue=True,
        param1=0,
        param2=0,
        param3=0,
        param4=10,
        x_lat=591727301,
        y_long=1029503488,
        z_alt=0,
    )  # Fly to position, keep current altitude
    wp_2 = Waypoint(
        frame=3,
        command=16,
        is_current=False,
        autocontinue=True,
        param1=0,
        param2=0,
        param3=0,
        param4=10,
        x_lat=592823149,
        y_long=1050352668,
        z_alt=0,
    )  # Fly to position, keep current altitude
    wp_3 = Waypoint(
        frame=3, command=20, is_current=False, autocontinue=True
    )  # Return to launch point
    wp_4 = Waypoint(
        frame=3,
        command=21,
        is_current=False,
        autocontinue=True,
        x_lat=591727301,
        y_long=1029503488,
        z_alt=0,
    )  # Land at same coordinates as wp_1
    mission_wp = [wp_0, wp_1, wp_2, wp_3, wp_4]
    mission_service_object = mission_push_client(
        0, mission_wp
    )  # Full waypoint update with waypoints in mission_wp
    print("Sent mission to vehicle: ", mission_service_object)

# # WaypointPushService
# Send waypoints to device
#
#  :start_index: will define a partial waypoint update. Set to 0 for full update
#
# Returns success status and transfered count

# uint16 start_index
# mavros_msgs/Waypoint[] waypoints
# ---
# bool success
# uint32 wp_transfered
