#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from spark_msgs.msg import ObjectDistances, BehaviourState
from std_msgs.msg import Int8

from go_state import GoState
from stop_state import StopState
from parking_state import ParkingState
from stopped_state import StoppedState

import time

class PlannerNode:

    def __init__(self):
        rospy.loginfo("BehaviourPlannerNode has started")

        self.init_listener = rospy.Subscriber("/behaviour_planner/init_signal", Int8, self.handle_init)

        # sub sign
        self.sign_subscriber = rospy.Subscriber(
            "/object_distances",
            ObjectDistances,
            self.update_signs_callback
        )

        '''self.road_subsriber = rospy.subscriber(
            "topic",
            "type",
            self.update_roads_callback
        )'''

        self.action_publisher = rospy.Publisher(
            "/behaviour_planner",
            BehaviourState,
            queue_size=1,
            latch=True
        )

        try:
            behaviours_id = rospy.get_param("/behaviour_states")
            traffic_sign_list = rospy.get_param("/darknet_ros/yolo_model/detection_classes/names")
        except KeyError as e:
            rospy.logerr(e)
            rospy.signal_shutdown("Params not set")
        
        self.planner_state_dict = {"parking": ParkingState(self.switch_state, self.action_publisher, behaviours_id, traffic_sign_list),
                                    "go": GoState(self.switch_state, self.action_publisher, behaviours_id, traffic_sign_list),
                                    "stop": StopState(self.switch_state, self.action_publisher, behaviours_id, traffic_sign_list),
                                    "stopped": StoppedState(self.switch_state, self.action_publisher, behaviours_id, traffic_sign_list)}
        
        rospy.loginfo("Waiting for subscriber to action..")
        while(self.action_publisher.get_num_connections() == 0):
            pass        
        rospy.loginfo("A subscriber is attached to action publisher, proceeding..")
        self.current_state = self.planner_state_dict["stopped"]
        self.current_state.state_switched()
 
    def handle_init(self, msg):
        rospy.loginfo("Coming here..")
        if msg.data == 1:
            self.current_state._switchToGo()

    def update_signs_callback(self, msg):
        sign_distance_dict = {msg.classes[i].data: msg.poses[i].x for i in range(len(msg.classes))} #decode msg
        for sign in sign_distance_dict:
            rospy.loginfo(sign)
        self.current_state.update_signs(sign_distance_dict)

    def update_roads_callback(self, msg):
        available_road_list = [] #decode msg
        self.current_state.update_roads(msg)

    def switch_state(self, state, indefinitely=False):
        self.current_state = self.planner_state_dict[state]
        if indefinitely:
            self.current_state.state_switched_indefinitely()
            return
        self.current_state.state_switched()

if __name__ == "__main__":
    rospy.init_node("BehaviourPlanner")
    bp = PlannerNode()
    rospy.spin()
