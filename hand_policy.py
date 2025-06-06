import numpy as np
from collections import defaultdict
from pid import PID


class HandTrackingWithPID:
    """
    A policy that follows a live hand path demonstration using PID control.

    Args:
        index_point: position of index fingertip
        thumb_point: position of thumb tip
        TBD
    """
    def __init__(self, robot_base_eef, dt=0.01):

        self.stage = 0

        self.index_origin = None
        self.curr_target = robot_base_eef
        self.robot_base = robot_base_eef
        self.depth_dist = 0.0

        self.calibrated = False
        self.depth_calibrated = False

        kp = 1.0 
        ki = 0.1
        kd = 1.0

        self.pid = PID(kp, ki, kd, robot_base_eef)


    def get_action(self, params, robot_eef_pos: np.ndarray) -> np.ndarray:
        """
        Compute next action for the robot's end-effector.

        Args:
            params: current hand position parameters obtained through CV model
            robot_eef_pos (np.ndarray): Current end-effector position [x,y,z].

        Returns:
            np.ndarray: Action vector [dx,dy,dz,0,0,0,grasp].
        """

        GRIP_DIST_THRESH = 0.15
        OBJ_DIST_THRESH = 0.05
        FORWARD_THRESH = 0.5
        BACKWARD_THRESH = 0.3

        action = np.zeros(7)

        if params != None:
            index = params[0]
            thumb = params[1]
            grip_dist = params[2]
            calibrated = params[3]
            lhs = params[4]

            if self.stage == 0:
                if calibrated:

                    self.stage = 1
                    self.index_origin = index 
                    self.calibrated = True
                    new_pos = self.robot_base.copy()

                    index_origin_x_dist = index[2] - self.index_origin[2]
                    index_origin_y_dist = index[0] - self.index_origin[0]
                    index_origin_z_dist = index[1] - self.index_origin[1]

                    new_pos[0] -= index_origin_x_dist*2
                    new_pos[1] -= index_origin_y_dist*.75
                    new_pos[2] -= index_origin_z_dist*.75

                    self.curr_target = new_pos
                    self.pid.reset(new_pos)


            if self.stage == 1:
                eef_dist = np.linalg.norm(robot_eef_pos - self.curr_target)   

                new_pos = self.robot_base.copy()               

                if eef_dist < OBJ_DIST_THRESH:
                    new_pos = self.robot_base.copy()   

                    index_origin_x_dist = index[2] - self.index_origin[2]
                    index_origin_y_dist = index[0] - self.index_origin[0]
                    index_origin_z_dist = index[1] - self.index_origin[1]

                    new_pos[0] -= index_origin_x_dist*2
                    new_pos[1] -= index_origin_y_dist*.75
                    new_pos[2] -= index_origin_z_dist*.75

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
        