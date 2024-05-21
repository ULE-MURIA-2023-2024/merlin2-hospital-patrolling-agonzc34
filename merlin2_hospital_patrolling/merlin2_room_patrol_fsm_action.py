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

from merlin2_hospital_patrolling.pddl import patrolled_wp


class Merlin2RoomPatrolAction(Merlin2FsmAction):
    """Merlin2 Navigation Action Class"""

    def __init__(self) -> None:

        self.__wp = PddlObjectDto(wp_type, "wp")

        super().__init__("check_wp")

        prepare_goal_state = CbState(["valid"], self.prepare_goal)
        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state("PREPARING_GOAL", prepare_goal_state, {"valid": "CHECKING_WP"})
        self.add_state("CHECKING_WP", tts_state)
        

    def prepare_goal(self, blackboard: Blackboard) -> str:
        blackboard.text = (
            f"Patrolling Waypoint {blackboard.merlin2_action_goal.objects[0][-1]}"
        )
        self.get_logger().info(blackboard.text)
        
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
