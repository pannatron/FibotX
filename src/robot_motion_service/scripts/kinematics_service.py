#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from robot_motion_service.srv import SolveIK
from kinematics import FiboX_Borot

class KinematicsService(Node):
    def __init__(self):
        super().__init__('kinematics_service')
        
        # Create the kinematics solver
        self.kinematics = FiboX_Borot()
        
        # Create the service
        self.service = self.create_service(
            SolveIK, 
            'solve_ik', 
            self.solve_ik_callback
        )
        
        self.get_logger().info('Kinematics Service is ready')
    
    def solve_ik_callback(self, request, response):
        """Handle inverse kinematics service requests."""
        try:
            # Extract the target position and orientation from the request
            x = request.x
            y = request.y
            z = request.z
            roll = request.rx
            pitch = request.ry
            yaw = request.rz
            
            self.get_logger().info(f'Received IK request: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}')
            
            # Compute inverse kinematics
            ik_solutions = self.kinematics.compute_ink(x, y, z, roll, pitch, yaw)
            
            if not ik_solutions:
                response.success = False
                response.message = "No IK solution found for the given pose"
                response.joint_angles = []
                return response
            
            # Get the first solution (we could implement a selection strategy here)
            best_solution = ik_solutions[0]
            
            # Convert radians to degrees for the response
            joint_angles_degrees = [np.degrees(angle) for angle in best_solution]
            
            response.success = True
            response.message = "IK solution found successfully"
            response.joint_angles = joint_angles_degrees
            
            self.get_logger().info(f'Found IK solution: {joint_angles_degrees}')
            
        except Exception as e:
            self.get_logger().error(f'Error solving IK: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
            response.joint_angles = []
        
        return response

def main():
    rclpy.init()
    node = KinematicsService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
