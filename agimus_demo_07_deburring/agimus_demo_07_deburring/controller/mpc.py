import time

from agimus_controller.mpc import MPC
from agimus_controller.mpc_data import OCPResults
from agimus_controller.trajectory import TrajectoryPoint


class DeburringMPC(MPC):
    def __init__(self) -> None:
        super().__init__()
        self._references_updated = False
        self._reference_trajectory = None

    def run(self, initial_state: TrajectoryPoint, _: int) -> OCPResults:
        assert self._ocp is not None
        assert self._warm_start is not None
        timer1 = time.perf_counter_ns()

        # Ensure that you have enough data in the buffer.
        if not self._references_updated:
            return None
        timer2 = time.perf_counter_ns()

        # TODO avoid building this list by making warm start classes use a reference trajectory with weights.
        reference_trajectory_points = [el.point for el in self._reference_trajectory]
        x0, x_init, u_init = self._warm_start.generate(
            initial_state, reference_trajectory_points
        )
        assert len(x_init) == self._ocp.n_controls + 1
        assert len(u_init) == self._ocp.n_controls

        timer3 = time.perf_counter_ns()
        self._ocp.solve(x0, x_init, u_init)
        self._warm_start.update_previous_solution(self._ocp.ocp_results)
        self._buffer.clear_past()
        timer4 = time.perf_counter_ns()

        # Extract the solution.
        self._mpc_debug_data.ocp = self._ocp.debug_data
        self._mpc_debug_data.reference_id = reference_trajectory_points[0].id
        self._mpc_debug_data.duration_iteration_ns = timer4 - timer1
        self._mpc_debug_data.duration_horizon_update_ns = timer2 - timer1
        self._mpc_debug_data.duration_generate_warm_start_ns = timer3 - timer2
        self._mpc_debug_data.duration_ocp_solve_ns = timer4 - timer3

        return self._ocp.ocp_results

    def update_references(self) -> None:
        # Ensure that you have enough data in the buffer.
        if len(self._buffer) < self._ocp.n_controls + 1:
            return None
        self._reference_trajectory = self._extract_horizon_from_buffer()
        self._ocp.set_reference_weighted_trajectory(self._reference_trajectory)
        self._references_updated = True
