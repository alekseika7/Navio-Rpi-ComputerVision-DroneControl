#!python3

import rospy
from std_msgs.msg import String

from Vehicle import Vehicle


def find_command(command):
    if command == "start_command":
        return drone.start_command
    elif command == "forward":
        return drone.forward
    elif command == "backward":
        return drone.backward
    elif command == "up":
        return drone.up
    elif command == "down":
        return drone.down
    elif command == "left":
        return drone.left
    elif command == "right":
        return drone.right
    elif command == "clockwise":
        return drone.clockwise
    elif command == "counterclockwise":
        return drone.counterclockwise
    elif command == "takeoff":
        return drone.takeoff
    elif command == "landing":
        return drone.landing
    elif command == "end_command":
        return drone.end_command
    else:
        rospy.loginfo("Unkmown command: %s" % command)
        return None


def process(command):
    global drone, cur_command, cur_command_func
    if command.data != cur_command:
        cur_command = command.data
        command_func = find_command(cur_command)
        if command_func is not None:
            command_func()
            cur_command_func = command_func
    else:
        cur_command_func()


if __name__ == "__main__":
    try:
        drone = Vehicle()
        cur_command = ""
        last_command = ""
        cur_command_func = None
        drone.arm()
        rospy.init_node("handler_node")
        rospy.Subscriber("/command_from_bs", String, process)
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
    finally:
        drone.disarm()

