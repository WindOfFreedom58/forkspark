#!/usr/bin/python

from abstract_state import AbstractState


class ParkingState(AbstractState):

    def __init__(self, switch_state_func, action_publisher, behaviours_id, traffic_sign_list):
        super().__init__(switch_state_func, action_publisher, behaviours_id, traffic_sign_list)

    def state_switched(self):
        self.publish_behaviour(self.behaviours_id["park_state"])
        
    def update_roads(self, roads):
        pass

    def update_signs(self, signs):
        # TODO: think about whether this should stay or not
        pass

    def make_decision(self, signs):
        pass

