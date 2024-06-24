import copy
import math
from enum import Enum

import numpy as np
import matplotlib.pyplot as plt

# commonroad
from commonroad.geometry.shape import Circle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory



# own code base
import curvilinear_cartesian as curv_cart

# typing
from typing import List, Tuple

# globals
_LABEL = "measured"    # "measured"


class _COLORS(Enum):
    """
    Colors for plotting
    """
    TUM_BLUE = "#0065BD"
    TUM_ORANGE = "#E37222"
    TUM_GREEN = "#A2AD00"
    TUM_BEIGE = "#DAD7CB"


class PlannedDrivenAssignment:
    """
    Assignment between planned and driven trajectory messages.
    Each initial pose msg of a planner gets assigned the temporal next FUTURE actual driving msg.
    """
    def __init__(
            self,
            planned_trajectory: CRTrajectory,
            planned_state: CustomState,
            planned_trajectory_idx: int,
            driven_state: CustomState,
            driven_state_idx: int,
            planning_dt: float=0.1
    ) -> None:


        self._planned_ros2_time: float = tuple_to_float(planned_state.ros2_time_stamp)
        self._planning_dt: float = planning_dt

        # states
        self._planned_trajectory = planned_trajectory
        self._planned_state: CustomState = planned_state
        self._planned_trajectory_idx: int = planned_trajectory_idx
        self._driven_state: CustomState = driven_state
        self._driven_state_idx: int = driven_state_idx

        # errors
        self._position_error: float = None
        self._orientation_error: float = None
        self._velocity_error: float = None
        self._compute_planned_driven_errors()

        # information about next state timing
        self._num_planning_states_to_next_step: int = None
        self._inter_replan_planning_states: List[CustomState] = None

        # information about inter planner steps
        self._inter_replan_comparison: List[PlannedDrivenAssignment] = list()

    def find_inter_replan_planning_states(self, next_planned_time: float) -> None:
        """
        Internally gets the indices ready for how many of the planner trajectory points were used
        until the next planning state cycle started.
        """
        delta_t: float = next_planned_time - self._planned_ros2_time
        self._num_planning_states_to_next_step: int = math.floor(delta_t / self._planning_dt)
        try:
            self._inter_replan_planning_states: List[CustomState] = [
                self._planned_trajectory.state_list[i + 1] for i in range(self._num_planning_states_to_next_step)
            ]
        except:
            self._inter_replan_planning_states = []


    def append_inter_replan_comparison(self, planned_driven_assignment) -> None:
        """
        Append inter-replan comparison
        """
        self._inter_replan_comparison.append(planned_driven_assignment)

    def _compute_planned_driven_errors(self) -> None:
        """
        Computes error between planned and driven states
        """
        self._position_error: float = math.sqrt(
            (self._planned_state.position[0] - self._driven_state.position[0]) ** 2 +
            (self._planned_state.position[1] - self._driven_state.position[1]) ** 2
        )

        self._orientation_error: float = self._planned_state.orientation - self._driven_state.orientation
        self._velocity_error: float = self._planned_state.velocity - self._driven_state.velocity



def visualize_planned_vs_driven_trajectory(
    planned_trajectories: List[CRTrajectory],
    driven_trajectory: CRTrajectory,
    reference_trajectory: CRTrajectory,
    save_path: str,
    save_img: bool = False,
    planning_dt: float=0.1,
    linewidth: float=0.5,
    axis_font_size: float = 5,
    plot_font_size: float = 5,
    title_font_size: float = 5
)->None:
    """
    Plots differences between planned and driven trajectory
    """
    points = np.asarray([point.position for point in reference_trajectory.state_list])


    CLCS = curv_cart.create_curvilinear_coordinate_system(points)

    # For each initial state of a planner trajectory, find the temporal next future state in the
    # driven trajectory --> Necessary, since the two have different ROS frequencies.
    planned_driven_assignments: List[PlannedDrivenAssignment] = list()
    for planned_idx, planned_trajectory in enumerate(planned_trajectories):
        if(planned_trajectory is None):
            continue
        planned_trajectory_init_state = planned_trajectory.state_list[0]
        planned_trajectory_time: float = tuple_to_float(planned_trajectory_init_state.ros2_time_stamp)

        # driven trajectory
        first_driven_state: CustomState = None
        for driven_idx, custom_state in enumerate(driven_trajectory.state_list):
            if(tuple_to_float(custom_state.ros2_time_stamp) >= planned_trajectory_time):
                first_driven_state = custom_state
                planned_driven_assignment = PlannedDrivenAssignment(planned_trajectory,
                                                                    planned_trajectory_init_state,
                                                                    planned_idx,
                                                                    first_driven_state,
                                                                    driven_idx,
                                                                    planning_dt=planning_dt)
                planned_driven_assignments.append(planned_driven_assignment)
                break

        if(first_driven_state is None):
            raise ValueError(f'Could not find a single '
                             f'driven trajectory state for planned state {planned_trajectory_init_state}')


    # Find out how many of the states of one planned trajectory were used before the next planning cycle
    for idx, pdass in enumerate(planned_driven_assignments):
        if(idx == 0):
            continue
        planned_driven_assignments[idx-1].find_inter_replan_planning_states(
            planned_driven_assignments[idx]._planned_ros2_time
        )

    planned_driven_assignments[-1].find_inter_replan_planning_states(
        tuple_to_float(driven_trajectory.state_list[-1].ros2_time_stamp)
    )


    # find planning states in between assignments
    for pdass_idx, pdass in enumerate(planned_driven_assignments):

        for planner_state_idx, planner_state in enumerate(pdass._inter_replan_planning_states):
            # calc planner time
            planner_time: float = tuple_to_float(planner_state.ros2_time_stamp) + pdass._planning_dt
            # driven trajectory
            first_driven_state: CustomState = None
            for driven_idx, custom_state in enumerate(driven_trajectory.state_list):
                # only take states after current
                if(driven_idx <= pdass._driven_state_idx):
                    continue
                if (tuple_to_float(custom_state.ros2_time_stamp) >= planner_time):
                    first_driven_state = custom_state
                    planned_driven_assignment = PlannedDrivenAssignment(pdass._planned_trajectory,
                                                                        custom_state,
                                                                        planner_state_idx,
                                                                        first_driven_state,
                                                                        driven_idx,
                                                                        planning_dt=planning_dt)
                    planned_driven_assignment._planned_ros2_time = planner_time
                    pdass.append_inter_replan_comparison(planned_driven_assignment)
                    break

            if (first_driven_state is None):
                raise ValueError(f'Could not find a single '
                                 f'driven trajectory state for planned state {planned_trajectory_init_state}')


    _visualize_planned_driven_assignment(planned_driven_assignments,
                                         save_path=save_path,
                                         CLCS=CLCS,
                                         save_img=save_img,
                                         linewidth=linewidth,
                                         axis_font_size=axis_font_size,
                                         title_font_size=title_font_size,
                                         plot_font_size=plot_font_size)






def _visualize_planned_driven_assignment(planned_driven_assignment: List[PlannedDrivenAssignment],
                                         save_path:str,
                                         CLCS,
                                         save_img: bool=False,
                                         linewidth=0.1,
                                         axis_font_size: float=1,
                                         title_font_size: float=5,
                                         plot_font_size: float=1) -> None:
    plt.locator_params(axis='y', nbins=3)

    time_list: List[float] = list()
    delta_t: float = planned_driven_assignment[0]._planned_ros2_time
    # Generate time sequence
    for pdass in planned_driven_assignment:
        time_list.append(pdass._planned_ros2_time - delta_t)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            time_list.append(inter_planning_pdass._planned_ros2_time - delta_t)

    time_list = [round(_time,3) for _time in time_list]


    # Position error plot
    plt.cla()
    plt.locator_params(axis='y', nbins=3)
    fig, plots = plt.subplots(nrows=3, ncols=1)
    # Position Error
    plots[0].set_title("Position Error", fontsize=title_font_size)
    p_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        p_error_list.append(pdass._position_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            p_error_list.append(inter_planning_pdass._position_error)
    plots[0].plot(time_list, p_error_list, color="black", linewidth=linewidth)
    #plots[0].legend()
    plots[0].set_xlabel("$t$ in $s$", fontsize=axis_font_size)
    plots[0].set_ylabel("$\Delta p$ in $m$", fontsize=axis_font_size)



    # Velocity Error
    plots[1].set_title("Velocity Error", fontsize=title_font_size)
    v_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        v_error_list.append(pdass._velocity_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            v_error_list.append(inter_planning_pdass._velocity_error)
    plots[1].plot(time_list, v_error_list, color="black", linewidth=linewidth)
    #plots[1].legend()
    plots[1].set_xlabel("$t$ in $s$", fontsize=axis_font_size)
    plots[1].set_ylabel("$\Delta v$ in $m/s$", fontsize=axis_font_size)

    # Orientation Error
    plots[2].set_title("Orientation Error", fontsize=title_font_size)
    orientation_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        orientation_error_list.append(pdass._orientation_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            orientation_error_list.append(inter_planning_pdass._orientation_error)
    plots[2].plot(time_list, orientation_error_list, color="black", linewidth=linewidth)
    #plots[2].legend()
    plots[2].set_xlabel("$t$ in $s$", fontsize=axis_font_size)
    plots[2].set_ylabel("$\Delta \Theta$ in $rad$", fontsize=axis_font_size)

    fig.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/physics_evaluation.svg", format="svg", bbox_inches="tight", transparent=False)

    ################## Same plots but each one a file ################################
    fig = plt.figure(figsize=(20,4))
    plt.cla()
    # Position Error
    plt.title("Position Error", fontsize=title_font_size)
    p_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        p_error_list.append(pdass._position_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            p_error_list.append(inter_planning_pdass._position_error)
    plt.plot(time_list, p_error_list, color="black", linewidth=linewidth)
    #plots[0].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$\Delta p$ in $m$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)


    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/_position_error.svg", format="svg", bbox_inches="tight", transparent=False)

    plt.cla()
    plt.locator_params(axis='y', nbins=3)
    # Position Error
    plt.title("Velocity Error", fontsize=title_font_size)
    v_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        v_error_list.append(pdass._velocity_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            v_error_list.append(inter_planning_pdass._velocity_error)
    plt.plot(time_list, v_error_list, color="black", linewidth=linewidth)
    #plots[1].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$\Delta v$ in $m/s$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)

    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/_velocity_error.svg", format="svg", bbox_inches="tight", transparent=False)

    # Position Error
    plt.cla()
    fig = plt.figure(figsize=(20,4))
    plt.title("Orientation Error", fontsize=title_font_size)
    orientation_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        orientation_error_list.append(pdass._orientation_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            orientation_error_list.append(inter_planning_pdass._orientation_error)
    plt.plot(time_list, orientation_error_list, color="black", linewidth=linewidth)
    #plots[2].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$\Delta \Theta$ in $rad$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[::3])

    fig.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/_orientation_error.svg", format="svg", bbox_inches="tight", transparent=False)


    ################## Velocity ################################

    # position trace
    plt.cla()
    fig = plt.figure(figsize=(20, 5))
    # Position Error
    #plt.title("Velocity Comparison", fontsize=title_font_size)
    v_soll: List[float] = list()
    v_ist: List[float] = list()
    for pdass in planned_driven_assignment:
        v_soll.append(pdass._planned_state.velocity)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            v_soll.append(inter_planning_pdass._planned_state.velocity)
    for pdass in planned_driven_assignment:
        v_ist.append(pdass._driven_state.velocity)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            v_ist.append(inter_planning_pdass._driven_state.velocity)
    plt.plot(time_list, v_soll, color=_COLORS.TUM_ORANGE.value, linewidth=linewidth, label="planned")
    plt.plot(time_list, v_ist, color=_COLORS.TUM_BLUE.value, linewidth=linewidth, linestyle="--", label=_LABEL)
    # plots[0].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$v$ in $m/s$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[::3])

    #plt.legend(loc="lower left", bbox_to_anchor=(0, 1.02, 1, 0.2),
    #           mode="expand", borderaxespad=0, ncol=2, fontsize=axis_font_size)

    fig.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/traces_velocity.svg", format="svg", bbox_inches="tight", transparent=False)

    ################# Orientation Trace #########################
    plt.cla()
    fig = plt.figure(figsize=(20, 5))
    #plt.title("Orientation Comparison", fontsize=title_font_size)
    orientation_soll: List[float] = list()
    orientation_ist: List[float] = list()
    for pdass in planned_driven_assignment:
        orientation_soll.append(pdass._planned_state.orientation)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            orientation_soll.append(inter_planning_pdass._planned_state.orientation)
    for pdass in planned_driven_assignment:
        orientation_ist.append(pdass._driven_state.orientation)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            orientation_ist.append(inter_planning_pdass._driven_state.orientation)
    plt.plot(time_list, orientation_soll, color=_COLORS.TUM_ORANGE.value, linewidth=linewidth, label="planned")
    plt.plot(time_list, orientation_ist, color=_COLORS.TUM_BLUE.value, linewidth=linewidth, linestyle="--", label=_LABEL)
    # plots[0].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$\Theta$ in $rad$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[::4])

    #plt.legend(loc="lower left", bbox_to_anchor=(0, 1.02, 1, 0.2),
    #           mode="expand", borderaxespad=0, ncol=2, fontsize=axis_font_size)



    fig.tight_layout()
    if (not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/traces_orientation.svg", format="svg", bbox_inches="tight", transparent=False)


    ################## P-Lon & P-lat Trace ######################################
    plt.cla()
    fig = plt.figure(figsize=(20, 5))
    #plt.title("Orientation Comparison", fontsize=title_font_size)
    lon_soll: List[float] = list()
    lon_ist: List[float] = list()
    lat_soll: List[float] = list()
    lat_ist: List[float] = list()

    for pdass in planned_driven_assignment:
        position_soll_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
            CLCS,
            pdass._planned_state.position[0],
            pdass._planned_state.position[1]
        )
        lon_soll.append(position_soll_cvl[0])
        lat_soll.append(position_soll_cvl[1])
        for inter_planning_pdass in pdass._inter_replan_comparison:
            position_soll_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
                CLCS,
                inter_planning_pdass._planned_state.position[0],
                inter_planning_pdass._planned_state.position[1]
            )
            lon_soll.append(position_soll_cvl[0])
            lat_soll.append(position_soll_cvl[1])

    for pdass in planned_driven_assignment:
        position_ist_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
            CLCS,
            pdass._driven_state.position[0],
            pdass._driven_state.position[1]
        )
        lon_ist.append(position_ist_cvl[0])
        lat_ist.append(position_ist_cvl[1])
        for inter_planning_pdass in pdass._inter_replan_comparison:
            position_ist_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
                CLCS,
                inter_planning_pdass._driven_state.position[0],
                inter_planning_pdass._driven_state.position[1]
            )
            lon_ist.append(position_ist_cvl[0])
            lat_ist.append(position_ist_cvl[1])

    plt.plot(time_list, lon_soll, color=_COLORS.TUM_ORANGE.value, linewidth=linewidth, label="planned")
    plt.plot(time_list, lon_ist, color=_COLORS.TUM_BLUE.value, linewidth=linewidth, linestyle="--", label=_LABEL)
    # plots[0].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$p_{lon}$ in $m$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[::3])

    plt.legend(loc="center", bbox_to_anchor=(0, 1.1, 1.0, 0.2),
               borderaxespad=0, ncol=2, fontsize=axis_font_size)
    fig.tight_layout()
    if (not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/traces_long.svg", format="svg", bbox_inches="tight", transparent=False)




    plt.cla()
    plt.locator_params(axis='y', nbins=3)
    plt.plot(time_list, lat_soll, color=_COLORS.TUM_ORANGE.value, linewidth=linewidth, label="planned")
    plt.plot(time_list, lat_ist, color=_COLORS.TUM_BLUE.value, linewidth=linewidth, linestyle="--", label=_LABEL)
    # plots[0].legend()
    plt.xlabel("$t$ in $s$", fontsize=axis_font_size)
    plt.ylabel("$p_{lat}$ in $m$", fontsize=axis_font_size)
    plt.xticks(fontsize=plot_font_size)
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[:3])

    #plt.legend(loc="lower left", bbox_to_anchor=(0, 1.02, 1, 0.2),
    #           mode="expand", borderaxespad=0, ncol=2, fontsize=axis_font_size)
    fig.tight_layout()
    if (not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/traces_lat.svg", format="svg", bbox_inches="tight", transparent=False)


    ################## Error boxplot plots #################################
    _boxplot_figsizes = (4, 8)
    _boxplot_widths = [0.75]

    plt.cla()
    fig = plt.figure(figsize=_boxplot_figsizes)
    # Veolcity
    #plt.title("Velocity Error Boxplot", fontsize=title_font_size)
    v_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        if(pdass._velocity_error != 0.0 and pdass._planned_ros2_time - delta_t > 20):
            v_error_list.append(pdass._velocity_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            if (inter_planning_pdass._velocity_error != 0.0 and inter_planning_pdass._planned_ros2_time - delta_t > 20):
                v_error_list.append(inter_planning_pdass._velocity_error)

    plt.boxplot(v_error_list, widths=_boxplot_widths)
    #plots[1].legend()
    plt.ylabel("$\Delta v$ in $m/s$", fontsize=axis_font_size)
    plt.xticks([])
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[[0,3,-1]])

    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/velocity_error_boxplot.svg", format="svg", bbox_inches="tight", transparent=False)

    # Orientation Error
    plt.cla()
    fig = plt.figure(figsize=_boxplot_figsizes)
    #plt.title("Orientation Error Boxplot", fontsize=title_font_size)
    orientation_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        if(pdass._planned_ros2_time - delta_t < 20):
            continue
        if(pdass._orientation_error != 0.0):
            orientation_error_list.append(pdass._orientation_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            if(pdass._orientation_error != 0.0):
                orientation_error_list.append(inter_planning_pdass._orientation_error)

    plt.boxplot(orientation_error_list, widths=_boxplot_widths)
    #plots[2].legend()
    plt.ylabel("$\Delta \Theta$ in $rad$", fontsize=axis_font_size)
    plt.xticks([])
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[[0,4,-1]])

    fig.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/orientation_error_boxplot.svg", format="svg", bbox_inches="tight", transparent=False)


    # Position Error
    plt.cla()
    fig = plt.figure(figsize=_boxplot_figsizes)
    #plt.title("Absolute Position Error (2-Norm) Boxplot", fontsize=title_font_size)
    p_error_list: List[float] = list()
    for pdass in planned_driven_assignment:
        if(pdass._planned_ros2_time - delta_t < 20):
            continue
        if(pdass._position_error != 0.0):
            p_error_list.append(pdass._position_error)
        for inter_planning_pdass in pdass._inter_replan_comparison:
            if (inter_planning_pdass._position_error != 0.0):
                p_error_list.append(inter_planning_pdass._position_error)

    plt.boxplot(p_error_list, widths=_boxplot_widths)
    #plots[1].legend()
    plt.ylabel("$\Delta p$ in $m$", fontsize=axis_font_size)
    plt.xticks([])
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[[0,3,-1]])

    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/abs_position_error_boxplot.svg", format="svg", bbox_inches="tight", transparent=False)


    #plt.title("Orientation Comparison", fontsize=title_font_size)
    lon_soll: List[float] = list()
    lon_ist: List[float] = list()
    lat_soll: List[float] = list()
    lat_ist: List[float] = list()

    for pdass in planned_driven_assignment:
        if(pdass._planned_ros2_time -delta_t < 20):
            continue
        position_soll_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
            CLCS,
            pdass._planned_state.position[0],
            pdass._planned_state.position[1]
        )
        lon_soll.append(position_soll_cvl[0])
        lat_soll.append(position_soll_cvl[1])
        for inter_planning_pdass in pdass._inter_replan_comparison:
            position_soll_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
                CLCS,
                inter_planning_pdass._planned_state.position[0],
                inter_planning_pdass._planned_state.position[1]
            )
            lon_soll.append(position_soll_cvl[0])
            lat_soll.append(position_soll_cvl[1])

    for pdass in planned_driven_assignment:
        if (pdass._planned_ros2_time - delta_t < 20):
            continue
        position_ist_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
            CLCS,
            pdass._driven_state.position[0],
            pdass._driven_state.position[1]
        )
        lon_ist.append(position_ist_cvl[0])
        lat_ist.append(position_ist_cvl[1])
        for inter_planning_pdass in pdass._inter_replan_comparison:
            position_ist_cvl = curv_cart.convert_point_cartesian_to_curvilinear(
                CLCS,
                inter_planning_pdass._driven_state.position[0],
                inter_planning_pdass._driven_state.position[1]
            )
            lon_ist.append(position_ist_cvl[0])
            lat_ist.append(position_ist_cvl[1])


    p_lon_error: List[float] = [
        p_lon_soll - p_lon_ist for p_lon_soll, p_lon_ist in zip(lon_soll, lon_ist)
        if p_lon_soll - p_lon_ist != 0
    ]
    p_lat_error: List[float] = [
        p_lat_soll - p_lat_ist for p_lat_soll, p_lat_ist in zip(lat_soll, lat_ist)
        if p_lat_soll - p_lat_ist != 0
    ]


    # Lon-pos Error
    plt.cla()
    fig = plt.figure(figsize=_boxplot_figsizes)
    #plt.title("Lon Position Error Boxplot", fontsize=title_font_size)

    plt.boxplot(p_lon_error, widths=_boxplot_widths)
    #plots[1].legend()
    plt.ylabel("$\Delta p$ in $m$", fontsize=axis_font_size)
    plt.xticks([])
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[[0,4,-1]])

    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/lon_position_error_boxplot.svg", format="svg", bbox_inches="tight", transparent=False)


    # Lon-pos Error
    plt.cla()
    #plt.title("Lat Position Error Boxplot", fontsize=title_font_size)

    plt.boxplot(p_lat_error, widths=_boxplot_widths)
    #plots[1].legend()
    plt.ylabel("$\Delta p$ in $m$", fontsize=axis_font_size)
    plt.xticks([])
    plt.yticks(fontsize=plot_font_size)
    ax = plt.gca()
    ax.set_yticks(ax.get_yticks()[[0,3,-1]])

    plt.tight_layout()
    if(not save_img):
        plt.show()
    else:
        plt.savefig(save_path + "/lat_position_error_boxplot.svg", format="svg", bbox_inches="tight", transparent=False)


def draw_car_state(renderer: MPRenderer,
            state: CustomState,
            point_radius: float=0.1):
    # draw center
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = "#000000"
    draw_params.shape.edgecolor = "#000000"
    draw_params.shape.opacity = 1.0
    occ_pos = Circle(radius=point_radius, center=state.position)
    occ_pos.draw(renderer, draw_params=draw_params)


def get_velocity_min_max_from_trajectory(trajectory:List[CustomState]) -> Tuple[float, float]:
    """
    Gets min and max velocity from global trajectory for color coding.
    """
    sorted_trajectory:List[CustomState] = sorted(trajectory, key=lambda x:x.velocity)
    min_velocity: float = sorted_trajectory[0].velocity
    max_velocity: float = sorted_trajectory[-1].velocity
    return (min_velocity, max_velocity)


def tuple_to_float(_tuple: Tuple[float, float]) -> float:
    """
    conversion to float ros time
    """
    time_converted: float = float(_tuple[0]) + float(_tuple[1])/1e9
    return time_converted