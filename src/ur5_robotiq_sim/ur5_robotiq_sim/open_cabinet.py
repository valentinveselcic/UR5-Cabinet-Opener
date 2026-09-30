#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time
import sys

# ROS Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_matrix

# MoveIt Messages & Services
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.action import MoveGroup 
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint

# Import klase Cabinet
from cabinet_generator.cabinet_model import Cabinet
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def quaternion_from_euler(roll, pitch, yaw):
    """
    Rucna konverzija u ROS Quaternion (float).
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))


class CabinetOpenerSolver(Node):
    def __init__(self):
        super().__init__('cabinet_opener_solver')

        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        self.get_logger().info('Cekam servise...')
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
             self.get_logger().error('Service compute_cartesian_path nedostupan.')
             return
        
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.get_logger().info('Sustav spreman.')
        

        # --- KONFIGURACIJA ---
        self.door_params = np.array([0.28, 0.35, 0.018, 0.3])
        self.spawn_pos = [0.0, 0.15, 0.998]
        self.spawn_yaw = -1.57
        self.initial_angle = -30.0 
        
        self.cabinet = Cabinet(
            door_params=self.door_params,
            axis_pos=-1, 
            T_A_S=np.eye(4), 
            initial_angle_deg=self.initial_angle
        )

        time.sleep(1.0)
        self.add_cabinet_obstacle()
        time.sleep(1.0)
    

    def add_cabinet_obstacle(self):
        
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "cabinet_body"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.3, 0.35, 0.37] 
        
        box_pose = Pose()
        box_pose.position.x = self.spawn_pos[0]
        box_pose.position.y = self.spawn_pos[1]
        
        box_pose.position.z = self.spawn_pos[2]#-0.02
    
        box_pose.orientation = quaternion_from_euler(0.0, 0.0, self.spawn_yaw)
        
        co.primitives.append(box)
        co.primitive_poses.append(box_pose)
        co.operation = CollisionObject.ADD
        
        for _ in range(5):
            self.collision_pub.publish(co)
            time.sleep(0.1)

        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "blocker"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.3, 0.2] 
        
        box_pose = Pose()
        box_pose.position.x = self.spawn_pos[0]
        box_pose.position.y = self.spawn_pos[1] - 0.5
        
        box_pose.position.z = 0.95
    
        box_pose.orientation = quaternion_from_euler(0.0, 0.0, self.spawn_yaw)
        
        co.primitives.append(box)
        co.primitive_poses.append(box_pose)
        co.operation = CollisionObject.ADD
        
        for _ in range(5):
            self.collision_pub.publish(co)
            time.sleep(0.1)

    def rot_z(self, angle_rad):
        s = np.sin(angle_rad)
        c = np.cos(angle_rad)
        R = np.eye(4)
        R[:3, :3] = np.array([[c, -s, 0],
                    [s, c, 0],
                    [0, 0, 1]])
        return R


    def calculate_path(self):
        c = np.cos(self.spawn_yaw)
        s = np.sin(self.spawn_yaw)
        T_O_W = np.eye(4)
        T_O_W[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        T_O_W[:3, 3] = np.array(self.spawn_pos)
        
        T_A_O = self.cabinet.T_A_O_init

        waypoints = []
        
        start_angle = self.initial_angle      
        target_angle = self.initial_angle - 75
        steps = 10
        angles = np.linspace(np.radians(start_angle), np.radians(target_angle), steps)

        for theta in angles:

            R_hinge = self.rot_z(theta)

            T_A_O_current = T_A_O @ R_hinge

            T_A_W = T_O_W @ T_A_O_current
            T_D_A = self.cabinet.T_D_A_init

            T_D_W = T_A_W @ T_D_A

            T_G_D = np.eye(4)

            # Gripper sa strane 
            # T_G_D[:3, :3] = np.array([ 
            #     [0, 0, -1],
            #     [0, 1, 0],
            #     [-1, 0, 0]
            # ])
            # T_G_D[:3, 3] = np.array([0.06, 0.08, 0.0])
            
            # Od gore
            T_G_D[:3, :3] = np.array([
                [1, 0, 0],
                [0, 0, 1],
                [0, 1, 0]
            ])
            T_G_D = T_G_D @ self.rot_z(np.radians(-45))

            # Offsetovi
            T_G_D[:3, 3] = np.array([-0.02, -0.17, 0.0])
            
            T_G_W = T_D_W @ T_G_D

            
            pose = Pose()
            pose.position.x = float(T_G_W[0, 3])
            pose.position.y = float(T_G_W[1, 3])
            pose.position.z = float(T_G_W[2, 3])
            
            q = quaternion_from_matrix(T_G_W)
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])

            waypoints.append(pose)

        if not waypoints:
            self.get_logger().error("Nisu generirani waypointi!")
            return
        start_pose = waypoints[0]
        self.get_logger().info(f"--- FAZA 1: PTP na Start (Angle {start_angle}) ---")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm" 
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        # goal_msg.request.max_velocity_scaling_factor = 0.1      
        # goal_msg.request.max_acceleration_scaling_factor = 0.1 

        cons = Constraints()
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.link_name = "tool0"
        pos_con.weight = 1.0
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05] 
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(start_pose)
        cons.position_constraints.append(pos_con)
        
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.link_name = "tool0"
        ori_con.orientation = start_pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        cons.orientation_constraints.append(ori_con)
        
        goal_msg.request.goal_constraints.append(cons)
        
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result() 
        
        if not goal_handle.accepted:
            self.get_logger().error("PTP Cilj odbijen od strane servera!")
            return

        self.get_logger().info("Cilj prihvaćen, čekam izvršavanje...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        final_result = result_future.result() 
        
    
        if final_result.result.error_code.val != 1:
            self.get_logger().error(f"PTP Neuspjeh! Error Code {final_result.result.error_code.val}, pokušavam ponovno...")
            self.calculate_path()  # Pokušaj ponovno
            return
        
        self.get_logger().info("PTP Uspješan. Robot je na startu.")
        time.sleep(1.0) 
        
        self.get_logger().info(f"--- FAZA 2: Cartesian Path do {target_angle} ---")

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = "arm"
        req.waypoints = waypoints
        req.max_step = 0.01     
        req.jump_threshold = 0.0 
        req.avoid_collisions = True 

        future_cart = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future_cart)
        response = future_cart.result()

        if response.fraction < 0.9:
            self.get_logger().error(f"Planiranje putanje neuspješno! Fraction: {response.fraction}, pokušavam ponovno...")
            self.calculate_path()  
            return

        self.get_logger().info(f"Putanja pronađena ({response.fraction * 100:.1f}%). Izvršavam...")
        
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = response.solution
        self.execute_client.send_goal_async(exec_goal)
        self.get_logger().info("Zahtjev za izvršavanje poslan.")


def main(args=None): 
    rclpy.init(args=args)
    node = CabinetOpenerSolver()
    try:
        node.calculate_path()
    except Exception as e:
        node.get_logger().error(f"Greska u izvrsavanju: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()