#!/usr/bin/env python3
from typing import List

import rclpy

from kant_dto import (
    PddlObjectDto,
    PddlConditionEffectDto,
)

from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at


from merlin2_fsm_action import Merlin2FsmAction, Merlin2BasicStates
from yasmin import CbState
from yasmin.blackboard import Blackboard
from geometry_msgs.msg import Twist
import time
from merlin2_hospital_patrolling.pddl import patrolled_wp


class Merlin2RoomPatrolAction(Merlin2FsmAction):
    """Merlin2 Navigation Action Class"""

    def __init__(self) -> None:

        self.__wp = PddlObjectDto(wp_type, "wp")

        super().__init__("patrol_wp")

        prepare_goal_state = CbState(["valid"], self.prepare_goal)
        patrol_state = CbState(["valid"], self.patrol_wp)
        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.patrol_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.add_state("PREPARING_GOAL", prepare_goal_state, {"valid": "PATROL_WP"})
        self.add_state("PATROL_WP", patrol_state, {"valid": "TTS_PATROL"})
        self.add_state("TTS_PATROL", tts_state)

    def prepare_goal(self, blackboard: Blackboard) -> str:
        print(blackboard.merlin2_action_goal.objects)
        blackboard.text = (
            f"Patrolled Waypoint {blackboard.merlin2_action_goal.objects[0][-1]}"
        )
        blackboard.start_time = time.time()

        self.get_logger().info(blackboard.text)
        return "valid"

    def patrol_wp(self, blackboard: Blackboard) -> str:
        patrol_msg = Twist()
        patrol_msg.angular.z = 0.3

        while time.time() - blackboard.start_time < 10:
            self.patrol_pub.publish(patrol_msg)
            time.sleep(1)
            
        patrol_msg.angular.z = 0
        self.patrol_pub.publish(patrol_msg)
        
        return "valid"

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self.__wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:
        condition_1 = PddlConditionEffectDto(
            robot_at, [self.__wp], time=PddlConditionEffectDto.AT_START
        )

        return [condition_1]

    def create_efects(self) -> List[PddlConditionEffectDto]:

        effect_1 = PddlConditionEffectDto(
            patrolled_wp, [self.__wp], time=PddlConditionEffectDto.AT_END
        )

        return [effect_1]


def main():
    rclpy.init()
    node = Merlin2RoomPatrolAction()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
