#!/usr/bin/python
import threading

from abstract_state import AbstractState


class StopState(AbstractState):

    def __init__(self, switch_state_func, action_publisher, behaviours_id, traffic_sign_list):
        super().__init__(switch_state_func, action_publisher, behaviours_id, traffic_sign_list)

    def state_switched(self):
        self.publish_behaviour(self.behaviours_id["stop_state"])
        self.waitThenGo(36)
    
    # Used when red light has been seen 
    def state_switched_indefinitely(self):
        self.publish_behaviour(self.behaviours_id["stop_state"])
        self.red_light_seen = True
        self.waitThenGoLight(20)

    def update_roads(self, roads):
        pass

    def waitThenGoLight(self, seconds):
        print(f"Waiting for {seconds} seconds..")
        timer = threading.Timer(seconds, self._switchToGoLight)
        timer.start()

    def _switchToGoLight(self):
        if self.red_light_seen:
            print("Switching to go state")
            self.red_light_seen = False
            self.switch_state_func("go")

    def waitThenGo(self, seconds):
        print(f"Waiting for {seconds} seconds..")
        timer = threading.Timer(seconds, self._switchToGo)
        timer.start()

    def _switchToGo(self):
        self.switch_state_func("go")
    
    def make_decision(self, signs):
        pass
