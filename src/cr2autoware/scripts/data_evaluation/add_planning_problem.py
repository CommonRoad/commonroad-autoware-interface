import os

# commonroad
from commonroad.scenario.state import InitialState
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.state import CustomState
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.common.util import Interval



def add_planning_problem(scenario_path: str,
                         save_path: str,
                         initial_state_of_pp: CustomState,
                         goal_state_of_pp: CustomState,
                         goal_width: float,
                         goal_length: float
                         ) -> None:
    """
    Adds planning problem to scenario and saves it

    :param scenario_path: path to commonroad xml
    :param save_path: path where commonroad xml is saved to
    :param initial_state_of_pp: initial state of planning problem
    :param goal_state_of_pp: goal state of planning problem
    :param goal_width: width of goal region in meters
    :param goal_length: length of goal region in meters
    """

    # Load commonroad scenario
    scenario, planning_problem_set = CommonRoadFileReader(
        scenario_path
    ).open()

    # remove existing obstacles (e.g. from a simulated scenario)
    for obstacle in scenario.obstacles:
        scenario.remove_obstacle(obstacle)

    # create initial state of pp
    initial_state: InitialState = InitialState(
        position=initial_state_of_pp.position,
        orientation=initial_state_of_pp.orientation,
        time_step=initial_state_of_pp.time_step,
        velocity=initial_state_of_pp.velocity,
        yaw_rate=0,
        slip_angle=0
    )

    # create goal region of pp
    region = Rectangle(
        length=goal_length,
        width=goal_width,
        center=goal_state_of_pp.position,
        orientation=goal_state_of_pp.orientation,
    )
    goal_state = CustomState(
        position=region,
        time_step=Interval(0, 1000),
        velocity=Interval(0,50)
    )
    goal_region = GoalRegion([goal_state])


    # create planning problem
    planning_problem = PlanningProblem(
        planning_problem_id=99999,
        initial_state=initial_state,
        goal_region=goal_region,
    )
    planning_problem_set = PlanningProblemSet()
    planning_problem_set.add_planning_problem(planning_problem)

    # add planning problem to scenario
    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=planning_problem_set,
        author="Edgar Data Generation",
        affiliation="Technical University of Munich",
        source="edgar drives"
    )

    # save file
    writer.write_to_file(
        os.path.join(save_path),
        OverwriteExistingFile.ALWAYS,
    )