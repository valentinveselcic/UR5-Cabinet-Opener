#!/usr/bin/env python3

from cabinet_generator.cabinet_model import Cabinet
import numpy as np
import os

def main():
    # Example dimensions
    door_params = np.array([0.28, 0.35, 0.018, 0.3]) # door width, door height, panel thickness, depth

    # Example rotation around Z
    T_A_W = np.eye(4)
    angle_deg = 150
    angle_rad = np.radians(angle_deg)
    T_A_W[:3,:3] = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad), 0],
        [np.sin(angle_rad),  np.cos(angle_rad), 0],
        [0, 0, 1]
    ])

    T_A_W[:3, 3] = np.array([-0.3, -0.4, 0.278])

    save_path ="/home/user/nrs_ws/cabinet.urdf"

    print("Generating cabinet...")
    cabinet = Cabinet(
        door_params=door_params,
        axis_pos=-1,
        r=np.array([0.0, -0.5 * door_params[0]]),
        save_path=save_path,
        has_handle=False,
        initial_angle_deg=30.0
    )

    print(f"Cabinet URDF saved to: {save_path}")

if __name__ == '__main__':
    main()