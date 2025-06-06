import numpy as np
from collections import defaultdict
from pid import PID


class HandTrackingWithPID:
    """
    A policy that follows a live hand path demonstration using PID control.

    Args:
        robot_base_eef (np.ndarray): the position of the robot's end effector on startup of the simulation environment
    """
    def __init__(self, robot_base_eef, dt=0.01):

        self.stage = 0

        self.index_origin_depth = None
        self.index_origin_lateral = None 
        self.robot_base_depth = robot_base_eef 
        self.robot_base_lateral = robot_base_eef
        self.current_mode = "lateral"

        self.curr_target = robot_base_eef
        self.robot_base = robot_base_eef

        self.calibrated = False

        kp = 1.0 
        ki = 0.1
        kd = 1.0

        self.pid = PID(kp, ki, kd, robot_base_eef)

    def switch_mode(self, new_mode, index, robot_eef_pos):
        """
        Switch mode based on distance of index finger and middle finger. Preserve current depth and lateral positions.

        Args:
            new_mode (string): mode to change to (either 'depth' or 'lateral')
            index (np.ndarray): current position of index finger 
            robot_eef_pos (np.ndarry): current position of the robot's end-effector
        """

        if self.current_mode != new_mode:
            self.current_mode = new_mode

            if new_mode == "depth":
                self.index_origin_depth = index.copy()
                self.robot_base_depth = robot_eef_pos.copy()
            elif new_mode == "lateral":
                self.index_origin_lateral = index.copy()
                self.robot_base_lateral = robot_eef_pos.copy()
                self.robot_base_depth = robot_eef_pos.copy()


    def get_action(self, params, robot_eef_pos: np.ndarray) -> np.ndarray:
        """
        Compute next action for the robot's end-effector.

        Args:
            params: current hand position parameters obtained through CV model
            robot_eef_pos (np.ndarray): current end-effector position [x,y,z].

        Returns:
            np.ndarray: Action vector [dx,dy,dz,0,0,0,grasp].
        """

        GRIP_DIST_THRESH = 0.15
        OBJ_DIST_THRESH = 0.05
        DEPTH_THRESH = 0.1

        action = np.zeros(7)

        if params != None:
            index = params[0]
            grip_dist = params[2]
            depth_dist = params[3]
            calibrated = params[4]

            if self.stage == 0:
                if calibrated:

                    self.stage = 1
                    self.index_origin_depth = index 
                    self.index_origin_lateral = index
                    self.calibrated = True
                    new_pos = self.robot_base.copy()

                    index_origin_y_dist = index[0] - self.index_origin_lateral[0]
                    index_origin_z_dist = index[1] - self.index_origin_lateral[1]

                    new_pos[1] -= index_origin_y_dist*.75
                    new_pos[2] -= index_origin_z_dist*.75

                    self.curr_target = new_pos
                    self.pid.reset(new_pos)


            if self.stage == 1:

                eef_dist = np.linalg.norm(robot_eef_pos - self.curr_target)
                new_pos = self.robot_base.copy() 

                if eef_dist < OBJ_DIST_THRESH: 

                    if depth_dist < DEPTH_THRESH:
                        self.switch_mode("depth", index, robot_eef_pos)
                        new_pos = self.robot_base_depth.copy()

                    else:
                        self.switch_mode("lateral", index, robot_eef_pos)
                        new_pos = self.robot_base_lateral.copy()

                    if self.current_mode == "depth":
                        index_origin_x_dist = index[0] - self.index_origin_depth[0]
                        new_pos[0] -= index_origin_x_dist
                    
                    else:

                        index_origin_y_dist = index[0] - self.index_origin_lateral[0]
                        index_origin_z_dist = index[1] - self.index_origin_lateral[1]

                        new_pos[1] -= index_origin_y_dist*.75
                        new_pos[2] -= index_origin_z_dist*.75
                        new_pos[0] = self.robot_base_depth[0]

                self.curr_target = new_pos
                self.pid.reset(new_pos)

            ctrl_output = self.pid.update(robot_eef_pos)
            action[0:3] = ctrl_output        
            if grip_dist < GRIP_DIST_THRESH:
                action[6] = 1
                self.grip = 1
            else:
                action[6] = -1
                self.grip = 0

        return action
        