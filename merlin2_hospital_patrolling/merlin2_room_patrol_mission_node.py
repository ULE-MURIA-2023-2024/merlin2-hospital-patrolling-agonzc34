#!/usr/bin/env python3
import rclpy
import yaml

from merlin2_mission import Merlin2FsmMissionNode

# states
from yasmin import CbState
from yasmin.blackboard import Blackboard

# pddl
from kant_dto import PddlObjectDto, PddlPropositionDto
from merlin2_hospital_patrolling.pddl import patrolled_wp
from merlin2_basic_actions.merlin2_basic_types import (
    wp_type,
)
from merlin2_basic_actions.merlin2_basic_predicates import (
    robot_at,
)

class Merlin2RoomPatrolMission(Merlin2FsmMissionNode):

    END = "end"
    NEXT = "next"

    def __init__(self) -> None:
        self.n_waypoints = 6
        
        super().__init__("room_patrol_node", run_mission=False, outcomes=[self.END])

        self.add_state(
            "RUN_PATROLLING",
            CbState([self.END], self.run_patrol),
            {self.END: self.END},
        )
        

    def create_objects(self):
        objects = []
        
        self.start_wp = PddlObjectDto(wp_type, 'start_wp')
        objects.append(self.start_wp)
        
        for i in range(self.n_waypoints):
            objects.append(PddlObjectDto(wp_type, f'wp{i}'))

        return objects

    def create_propositions(self):
        propositions = []
        
        propositions.append(PddlPropositionDto(robot_at, [self.start_wp], is_goal=False))

        return propositions

    def move_robot_to_init_pose(self, blackboard: Blackboard) -> str:
        goal = PddlPropositionDto(robot_at, [self.start_wp], is_goal=True)
        self.execute_goal(goal)
        return self.END

    def run_patrol(self, blackboard: Blackboard) -> str:
        goals = []
        
        objects_iter = iter(self.create_objects())
        next(objects_iter)

        for wp_obj in objects_iter:
            goals.append(PddlPropositionDto(patrolled_wp, [wp_obj], is_goal=True))
            
        self.execute_goals(goals)
        return self.END


def main():
    rclpy.init()
    node = Merlin2RoomPatrolMission()
    node.execute_mission()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
