from pathlib import Path
from typing import Dict

import crcpp
import crmonitor
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleType


def main() -> None:
    base_path = Path("/home/lercher/tum/edgar/data/artifact")
    experiments = [
        "2025-04-11_first_test_safe_dist",
        "2025-04-11_second_test_safe_dist",
        "2025-04-11_left_turn",
        "2025-04-16_first_test_safe_dist",
        "2025-04-16_second_test_safe_dist",
        "2025-04-16_right_turn",
    ]
    results = {}
    for experiment in experiments:
        exp_path = base_path / experiment / "CommonRoad"
        scenario_path = exp_path / "scenario_obstacles_ego_interpolated.xml"
        results[experiment] = monitor_scenario(scenario_path)
    for experiment in experiments:
        print(f"Experiment: {experiment}")
        for rule, result in results[experiment].items():
            print(f"  {rule}: {'Satisfied' if result else 'Violated'}")


def monitor_scenario(scenario_path: Path) -> Dict[str, bool]:
    scenario, _ = CommonRoadFileReader(scenario_path).open()
    scenario.remove_obstacle(
        [obs for obs in scenario.obstacles if obs.obstacle_type == ObstacleType.UNKNOWN]
    )

    predicate_cost = {}
    temporal_parameters = {}
    predicate_parameter = {
        "roadConditionSpeedLimit": 6.9,
    }
    monitor_config = crmonitor.MonitorConfiguration()
    sim_param = crmonitor.MonitorSimulationParameters()
    # turn off monitor evaluation settings
    sim_param.performanceMeasurement = False
    sim_param.storeEvalResults = False
    # Next three options make sure that no obstacles are excluded from the world
    sim_param.checkEgoValid = False
    sim_param.checkObstacleValid = False
    sim_param.removeVRU = False
    wp = crcpp.WorldParameters(
        crcpp.RoadNetworkParameters(),
        crcpp.SensorParameters(250.0, 250.0),
        crcpp.ActuatorParameters.ego_defaults(),
        crcpp.TimeParameters(101, 1.5, scenario.dt),
        crcpp.ActuatorParameters.vehicle_defaults(),
    )
    sim_param.world_parameters = wp
    rule_monitor = crmonitor.Monitor()
    rule_monitor.set_config(
        sim_param,
        monitor_config,
        predicate_cost,
        predicate_parameter,
        temporal_parameters,
    )
    rule_monitor.activate_rule_sets(["R_G1", "R_G2", "R_G3", "R_G4", "R_U1", "R_U2", "R_U3", "R_U4", "R_U5", "R_U7"])

    world = crcpp.World(scenario, wp)
    ego_id = 42
    result = rule_monitor.evaluate_scenarios_on_given_egos([world], [ego_id])
    return {
        rule: not rule_result or (ego_id in rule_result and not rule_result[ego_id])
        for rule, rule_result in result["ZAM_MUC2D-1"][ego_id].items()
    }


if __name__ == "__main__":
    main()
