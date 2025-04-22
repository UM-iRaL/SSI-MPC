""" Implementation of the data-augmented MPC for quadrotors.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""


import numpy as np
from src.quad_mpc.quad_3d_optimizer import Quad3DOptimizer
from src.utils.quad_3d_opt_utils import simulate_plant, uncertainty_forward_propagation
from src.utils.utils import make_bx_matrix


class Quad3DMPC:
    def __init__(self, my_quad, t_horizon=1.0, n_nodes=5, q_cost=None, r_cost=None,
                 optimization_dt=5e-2, simulation_dt=5e-4, model_name="my_quad", q_mask=None,
                 solver_options=None, rf_dict=None):
        """
        :param my_quad: Quadrotor3D simulator object
        :type my_quad: Quadrotor3D
        :param t_horizon: time horizon for optimization loop MPC controller
        :param n_nodes: number of MPC nodes
        :param optimization_dt: time step between two successive optimizations intended to be used.
        :param simulation_dt: discretized time-step for the quadrotor simulation
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 13.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 4.
        :param q_mask: Optional boolean mask that determines which variables from the state compute towards the
        cost function. In case no argument is passed, all variables are weighted.
        :param solver_options: Optional set of extra options dictionary for acados solver.
        """

        self.quad = my_quad
        self.simulation_dt = simulation_dt
        self.optimization_dt = optimization_dt

        # motor commands from last step
        self.motor_u = np.array([0., 0., 0., 0.])

        self.n_nodes = n_nodes
        self.t_horizon = t_horizon
        # For MPC optimization use
        self.quad_opt = Quad3DOptimizer(my_quad, t_horizon=t_horizon, n_nodes=n_nodes,
                                        q_cost=q_cost, r_cost=r_cost,
                                        model_name=model_name, q_mask=q_mask,
                                        solver_options=solver_options, rf_dict=rf_dict)

    def clear(self):
        self.quad_opt.clear_acados_model()

    def get_state(self):
        """
        Returns the state of the drone, with the angle described as a wxyz quaternion
        :return: 13x1 array with the drone state: [p_xyz, a_wxyz, v_xyz, r_xyz]
        """

        x = np.expand_dims(self.quad.get_state(quaternion=True, stacked=True), 1)
        return x

    def set_reference(self, x_reference, u_reference=None):
        """
        Sets a target state for the MPC optimizer
        :param x_reference: list with 4 sub-components (position, angle quaternion, velocity, body rate). If these four
        are lists, then this means a single target point is used. If they are Nx3 and Nx4 (for quaternion) numpy arrays,
        then they are interpreted as a sequence of N tracking points.
        :param u_reference: Optional target for the optimized control inputs
        """

        if isinstance(x_reference[0], list):
            # Target state is just a point
            return self.quad_opt.set_reference_state(x_reference, u_reference)
        else:
            # Target state is a sequence
            return self.quad_opt.set_reference_trajectory(x_reference, u_reference)

    def optimize(self, use_model=0, return_x=False, update_rff_dict=None):
        """
        Runs MPC optimization to reach the pre-set target.
        :param use_model: Integer. Select which dynamics model to use from the available options.
        :param return_x: bool, whether to also return the optimized sequence of states alongside with the controls.
        :param update_rff_dict: dict, dt and odom used to update rff weights.

        :return: 4*m vector of optimized control inputs with the format: [u_1(0), u_2(0), u_3(0), u_4(0), u_1(1), ...,
        u_3(m-1), u_4(m-1)]. If return_x is True, will also return a vector of shape N+1 x 13 containing the optimized
        state prediction.
        """

        quad_current_state = self.quad.get_state(quaternion=True, stacked=True)

        # Remove rate state for simplified model NLP
        out_out = self.quad_opt.run_optimization(quad_current_state, use_model=use_model, return_x=return_x, update_rff_dict=update_rff_dict)
        return out_out

    @staticmethod
    def reshape_input_sequence(u_seq):
        """
        Reshapes the an output trajectory from the 1D format: [u_0(0), u_1(0), ..., u_0(n-1), u_1(n-1), ..., u_m-1(n-1)]
        to a 2D n x m array.
        :param u_seq: 1D input sequence
        :return: 2D input sequence, were n is the number of control inputs and m is the dimension of a single input.
        """

        k = np.arange(u_seq.shape[0] / 4, dtype=int)
        u_seq = np.atleast_2d(u_seq).T if len(u_seq.shape) == 1 else u_seq
        u_seq = np.concatenate((u_seq[4 * k], u_seq[4 * k + 1], u_seq[4 * k + 2], u_seq[4 * k + 3]), 1)
        return u_seq

    def reset(self):
        return
