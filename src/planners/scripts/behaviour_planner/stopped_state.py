#!/usr/bin/python
import threading

from abstract_state import AbstractState


class StoppedState(AbstractState):

    def __init__(self, switch_state_func, action_publisher, behaviours_id, traffic_sign_list): 
        super().__init__(switch_state_func, action_publisher, behaviours_id, traffic_sign_list) 

    def state_switched(self):
        self.publish_behaviour(self.behaviours_id["stopped_state"])

    def update_roads(self, roads):
        pass

    def _switchToGo(self):
        print("Switching to go state")
        self.switch_state_func("go")
    
    def make_decision(self, signs):
        pass
