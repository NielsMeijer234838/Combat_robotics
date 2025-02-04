import pybullet as p
import pybullet_data
import numpy as np
import time


class Simulation:
    def __init__(self, render=False):
        # Connect to PyBullet
        if render:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load environment
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")

        # Load the robot
        self.robot_start_pos = [0, 0, 0.1]
        self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF("puncture_prime.urdf", self.robot_start_pos, self.robot_start_orientation)

        # Retrieve and store joint indices
        self.left_wheel_index = None
        self.right_wheel_index = None

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if b'left_wheel_hinge' in joint_info[1]:
                self.left_wheel_index = i
            elif b'right_wheel_hinge' in joint_info[1]:
                self.right_wheel_index = i

        if self.left_wheel_index is None or self.right_wheel_index is None:
            raise ValueError("Wheel joints not found in the robot definition.")

        # Get the link index of the pin
        self.pin_index = None
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if b'pin' in joint_info[12]:  # Check the link name
                self.pin_index = i

        if self.pin_index is None:
            raise ValueError("Pin link not found in the robot definition.")

    def reset_robot(self):
        p.resetBasePositionAndOrientation(self.robot_id, self.robot_start_pos, self.robot_start_orientation)
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        # Destroy balloon if it exists
        if hasattr(self, 'balloon_id'):
            p.removeBody(self.balloon_id)

    def set_wheel_velocity(self, left_velocity, right_velocity):
        p.setJointMotorControl2(self.robot_id, self.left_wheel_index, p.VELOCITY_CONTROL, targetVelocity=left_velocity)
        p.setJointMotorControl2(self.robot_id, self.right_wheel_index, p.VELOCITY_CONTROL, targetVelocity=right_velocity)

    def step_simulation(self):
        p.stepSimulation()
        self.check_collision()  # Check for collision in each simulation step
        sleep_time = 1 / 240
        time.sleep(sleep_time)

    def get_robot_state(self):
        pos, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler_orientation = p.getEulerFromQuaternion(orientation)
        return np.array([pos[0], pos[1], euler_orientation[2]])

    def close(self):
        p.disconnect()

    def render_balloon(self, target_pos):
        # Create and render a red balloon (30 cm diameter)
        balloon_radius = 0.15  # 30 cm diameter
        balloon_mass = 0.01
        balloon_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=balloon_radius)
        balloon_visual = p.createVisualShape(p.GEOM_SPHERE, radius=balloon_radius, rgbaColor=[1, 0, 0, 1])
        self.balloon_id = p.createMultiBody(
            baseMass=balloon_mass,
            baseCollisionShapeIndex=balloon_collision,
            baseVisualShapeIndex=balloon_visual,
            basePosition=[target_pos[0], target_pos[1], balloon_radius],
            baseOrientation=[0, 0, 0, 1]
        )

    def check_collision(self):
        # Check for collisions between the pin and the balloon
        if hasattr(self, 'balloon_id') and self.pin_index is not None:
            contact_points = p.getContactPoints(bodyA=self.robot_id, bodyB=self.balloon_id, linkIndexA=self.pin_index)
            if contact_points:
                print("Balloon popped!")
                p.removeBody(self.balloon_id)
                del self.balloon_id  # Remove the attribute to avoid repeated checks

    def get_balloon_position(self):
        if hasattr(self, 'balloon_id'):
            pos, _ = p.getBasePositionAndOrientation(self.balloon_id)
            return np.array(pos[:2])  # Return only x and y
        return np.array([0.0, 0.0])  # Default position if balloon doesn't exist
