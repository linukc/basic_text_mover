#! /usr/bin/env python3

import rospy
import actionlib
from communication_msgs.msg import SimpleCommandAction, SimpleCommandResult
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

class SimpleMove(object):
    # create messages that are used to publish feedback/result
    _result = SimpleCommandResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SimpleCommandAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.command_topic = rospy.get_param("basic_mover/cmd_topic")
        self.babysitter = rospy.get_param("basic_mover/use_babysitter")
        self.max_linear_velocity = rospy.get_param("basic_mover/max_linear_velocity")
        self.max_angular_velocity = rospy.get_param("basic_mover/max_angular_velocity")
        self.publish_rate = rospy.get_param("basic_mover/publish_rate")
        
        self.command_mapper = {
            "move_forward": Twist(linear=Vector3(self.max_linear_velocity, 0, 0)),
            "move_backward": Twist(linear=Vector3(-self.max_linear_velocity, 0, 0)),
            "turn_clockwise": Twist(angular=Vector3(0, 0, -self.max_angular_velocity)),
            "turn_counterclockwise": Twist(angular=Vector3(0, 0, self.max_angular_velocity))
        }

        self.rate = rospy.Rate(self.publish_rate)
        self.publisher = rospy.Publisher(self.command_topic, Twist, queue_size=10)
        
        if self.babysitter:
            raise NotImplementedError

    def calculate_movement(self, movement_command):
        try:
            tokens = movement_command.split("_")
            print(f"Get tokens:'{tokens}'")
            command, distance_contraint = "_".join(tokens[:-1]), float(tokens[-1])

        except:
            print(f"Aborting - can't parse input command '{movement_command}'")
            return None
            
        ros_msg = self.command_mapper[command]
        if command.startswith("move"):
            time_constraint = distance_contraint / self.max_linear_velocity
        elif command.startswith("turn"):
            time_constraint = 6.28 / (self.max_angular_velocity * (360 / distance_contraint))

        return ros_msg, time_constraint
      
    def execute_cb(self, goal):
        
        movement_task = self.calculate_movement(goal.command)
        if not movement_task:
            self._as.set_aborted()

        ros_msg, time_constraint = movement_task

        start_time = rospy.Time.now()
        movement_time = rospy.Duration(time_constraint)
        
        while rospy.Time.now() - start_time < movement_time:
            self.publisher.publish(ros_msg)

            if self._as.is_preempt_requested():
                print(f'Preempted')
                self._as.set_preempted()
            self.rate.sleep()
        
        self.publisher.publish(Twist())
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('simple_move_node')
    server = SimpleMove("simple_move_server")
    rospy.spin()