#!/usr/bin/python
from abc import abstractmethod, ABC
from sensor_msgs.msg import Joy
from spark_msgs.msg import BehaviourState

class AbstractState(ABC):

    def __init__(self, switch_state_func, action_publisher, behaviours_id, traffic_sign_list):
        self.switch_state_func = switch_state_func
        self.action_publisher = action_publisher
        self.behaviours_id = behaviours_id
        #self.sign_list = traffic_sign_list

        #self.min_sign_distance = 10.0
        self.road_session_count = 5

        self.sign_list = ["dur", # 0
                          "durak", # 1
                          "saga donulmez", # 2
                          "sola donulmez", # 3
                          "girilmez", # 4
                          "park yasak", # 5
                          "park", # 6
                          "kirmizi isik", # 7
                          "yesil isik", # 8
                          "isik yok", # 9
                          "ileriden sola mecburi", # 10
                          "ilerden saga mecburi", # 11
                          "ileri ve sag",# 12
                          "ileri ve sol"] # 13

        self.elimination_dict = {
            self.sign_list[2]: ["right"],
            self.sign_list[3]: ["left"],
            self.sign_list[4]: ["straight"],
            self.sign_list[10]: ["right", "straight"],
            self.sign_list[11]: ["left", "straight"],
            self.sign_list[12]: ["left"],
            self.sign_list[13]: ["right"]
        }

        """
        After what distance we will take into account the sign         
        """
        self.min_sign_distances = {
            self.sign_list[0]: 10.,
            self.sign_list[1]: 10.,
            self.sign_list[2]: 14.,
            self.sign_list[3]: 14.,
            self.sign_list[4]: 14.,  # Önemli threshold değiştirilebilir
            self.sign_list[7]: 11.,
            self.sign_list[8]: 8,
            self.sign_list[9]: 11,
            self.sign_list[10]: 8.5,
            self.sign_list[11]: 8.5,
            self.sign_list[12]: 10.,
            self.sign_list[13]: 10.
        }

        """
        After what distance the decision will be executed if the sign count 
        has exceeded the corresponding sign session count
        """
        self.decision_sign_distances = {
            self.sign_list[0]: 6,
            self.sign_list[1]: 6,
            self.sign_list[2]: 4,
            self.sign_list[3]: 4,
            self.sign_list[4]: 4, # Önemli threshold değiştirilebilir
            self.sign_list[7]: 8,
            self.sign_list[8]: 6,
            self.sign_list[9]: 8,
            self.sign_list[10]: 6,
            self.sign_list[11]: 6,
            self.sign_list[12]: 6,
            self.sign_list[13]: 6
        }

        """
        Number of signs to be counted before executing a decision with respect to the sign
        """
        self.sign_session_counts = {
            self.sign_list[0]: 7,
            self.sign_list[1]: 7,
            self.sign_list[2]: 15,
            self.sign_list[3]: 15,
            self.sign_list[4]: 15,  # Önemli threshold değiştirilebilir
            self.sign_list[5]: 7,
            self.sign_list[6]: 6,
            self.sign_list[7]: 7,
            self.sign_list[8]: 7,
            self.sign_list[9]: 7,
            self.sign_list[10]: 7,
            self.sign_list[11]: 7,
            self.sign_list[12]: 7,
            self.sign_list[13]: 7
            }

        #self.sign_dictations = ["stop", "eliminate_left", "eliminate_right", "eliminate_straight", "park", "left", "right"]
        self.road_list = ["left", "right", "straight"] # şimdilik
        self.available_roads = ["left", "right", "straight"] # şimdilik

        self.sign_dict = {sign: self.sign_session_counts[sign] for sign in self.sign_list}
        self.road_dict = {road: self.road_session_count for road in self.road_list}

        self.decision_made_dict = {}

        # keeps currently visible signs ahead of the vehicle with their respective distances
        self.current_signs = {}

        # keeps last signs that are used in decision making
        self.last_decision_signs = {}

        # keeps possible road paths that vehicle can follow.
        self.current_roads = []

        self.decision_signs = {}
        self.decision_processing = False
        self.decision_processing_checked = False
        self.red_light_seen = False

    # Takes lastly seen signs with their respective distances as dictionary.
    # eğer isik yok, yesil isik ve kirmizi isik'ten herhangi bir geliyorsa ve gelen şey yesil isik değilse dur(yani yesil ise hareket et)
    
    def update_signs(self, signs):

        if self.sign_list[8] in signs and self.red_light_seen:
            self.red_light_seen = False
            self.switch_state_func("go")

        # print("Decision made dict 1:", self.decision_made_dict)
        #
        # print("Decision signs first:", self.decision_signs)

        # reset last_decision_signs if decision is completely processed
        for decision_sign in self.last_decision_signs:
            if decision_sign in signs and 0 <= signs[decision_sign] <= self.last_decision_signs[decision_sign]:
                self.decision_processing = True
                break
            else:
                print("Decision made dict 2:", self.decision_made_dict)
                self.decision_made_dict[decision_sign] -= 1
                if self.decision_made_dict[decision_sign] == 0:
                    print("Geldi")
                    self.decision_processing = False

        # print("Decision processing 1:", self.decision_processing)
        # print("Decision processing checked:", self.decision_processing_checked)

        if not self.decision_processing:
            # print("Decision processing done")
            self.last_decision_signs = {}
            self.decision_signs = {}
            self.decision_made_dict = {}
            self.available_roads = self.road_list[:]
            self.decision_processing_checked = True

        # print("LAST DECISION SIGNS:", self.last_decision_signs)

        # gelecek decision'lara ekleme kısmı
        for sign, distance in signs.items():
            # if sign is farther than min_sign_distance or the sign is already considered in decision making ignore it
            if (distance > self.min_sign_distances[sign]) or \
                    (sign in self.last_decision_signs and 0 <= distance <= self.last_decision_signs[sign]):
                continue

            elif sign not in self.decision_signs:
                if not self.decision_processing:
                    self.sign_dict[sign] -= 1
                    if self.sign_dict[sign] == 0:
                        self.decision_signs[sign] = distance
            # update signs' distances that will later be considered in decision making
            else:
                self.decision_signs[sign] = distance

        if self.decision_processing:
            return

        # print("Decision processing 2:", self.decision_processing)

        # decision yapma kısmı
        for sign in self.decision_signs:
            # print("Sign 1:", sign)
            self.assign_decision_sign_distance(sign)
            if sign in signs and signs[sign] < self.decision_sign_distance and not self.decision_processing:
                # print("Sign 2:", sign)
                self.last_decision_signs = self.decision_signs.copy()
                self.decision_made_dict = {sign: self.sign_session_counts[sign] for sign in self.decision_signs}
                self.decision_processing_checked = False
                self.make_decision(self.decision_signs.keys())
                self.decision_processing = True
                break

    # Takes lastly seen possible roads that vehicle can follow.
    @abstractmethod
    def update_roads(self, roads):
        pass

    @abstractmethod
    def state_switched(self):
        pass

    @abstractmethod
    def make_decision(self,signs):
        pass

    def _switchToGo(self):
        pass

    def reset_sign_road_dicts(self):
        self.sign_dict = {sign: self.sign_session_counts[sign] for sign in self.sign_list}
        self.road_dict = {road: self.road_session_count for road in self.road_list}

        self.current_signs = {}
        self.current_roads = []
    
    def publish_behaviour(self, behaviour_id):
        behaviour_state = BehaviourState()
        #TODO: publish timestamp //
        #behaviour_state.header.stamp = 
        behaviour_state.state = behaviour_id
        # print(behaviour_state)
        self.action_publisher.publish(behaviour_state)
        # print("published..")

    def assign_decision_sign_distance(self, sign):
        self.decision_sign_distance = self.decision_sign_distances[sign]
