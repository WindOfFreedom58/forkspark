#!/usr/bin/python

from abstract_state import AbstractState
import threading


class GoState(AbstractState):

    def __init__(self, switch_state_func, action_publisher, behaviours_id, traffic_sign_list):
        super().__init__(switch_state_func, action_publisher, behaviours_id, traffic_sign_list)

        self.selected_road = "left"
        # self.decision_sign_distance = 11.0

    def eliminate_by_signs(self, signs):
        for sign in signs:
            if sign in self.elimination_dict:
                self.eliminate_roads(self.elimination_dict[sign])

    def eliminate_roads(self, roads):
        for road in roads:
            if road in self.available_roads:
                self.available_roads.remove(road)

    def recover_from_straight(self):
        self.publish_behaviour(self.behaviours_id["right" + "_state"])

    def state_switched(self):
        # print("Coming here")
        # print("STATE SWITCHED FROM GO STATE: " + self.selected_road)
        self.publish_behaviour(self.behaviours_id[self.selected_road + "_state"])
        
        if self.selected_road == "straight":
            timer = threading.Timer(16, self.recover_from_straight)
            timer.start()

    # Possible bug: bir signın varlığından emin olup onu listeye eklerken son framedeki distance alınıyor.
    # Fakat alınan distance eğer hatalı olur ise sıkıntı çıkabilir.
    def update_roads(self, roads):
        for road in roads:
            pass

    def _switchToGo(self):
        self.selected_road = "left"
        self.state_switched()

    def make_decision(self, signs):
        # print("MAKE DECISION")
        self.reset_sign_road_dicts()

        # If 'kirmizi isik', 'yesil isik' or 'isik yok' has been, seen state that a traffic light has been seen
        traffic_light_exists = False
        for sign in signs:
            if sign == self.sign_list[7] or \
                    sign == self.sign_list[8] or \
                    sign == self.sign_list[9]:
                traffic_light_exists = True
                break

        # Before checking any other sign, check if 'dur' or 'durak' exists in our decision signs
        if self.sign_list[0] in signs or self.sign_list[1] in signs:
            # print("STOPPED")
            self.switch_state_func("stop")
            self.reset_sign_road_dicts()
            return

        if self.sign_list[7] in signs:
            self.switch_state_func("stop", indefinitely=True)
            self.selected_road = "right"

        if self.sign_list[5] in signs or self.sign_list[6] in signs:
            self.switch_state_func("parking")

        # Eliminate the roads that are unaccepted due to the signs that we are currently using to make a decision
        self.eliminate_by_signs(signs)

        # print("Available roads:", self.available_roads)
        # If there is only one available road, then select it
        if len(self.available_roads) == 1:
            self.selected_road = self.available_roads[0]
        elif len(self.available_roads) == 2:
            if self.road_list[2] in self.available_roads:
                self.available_roads.remove(self.road_list[2])
                self.selected_road = self.available_roads[0]
            else:
                self.selected_road = "right"
        else:
            print("Decision cannot be made:", self.available_roads)

        # Publish the road that is switched to if not stopped
        # print("TRAFFIC LIGHT EXISTS:", traffic_light_exists)
        if self.sign_list[7] not in signs:
            self.state_switched()
        self.reset_sign_road_dicts()
