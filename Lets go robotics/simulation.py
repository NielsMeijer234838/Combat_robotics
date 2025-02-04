import pybullet as p
import pybullet_data
import time
import random
import numpy as np

class BalloonSimulation:
    def __init__(self):
        # Connect to the PyBullet GUI and setup simulation environment
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.robot_id = None
        self.balloons = []
        print("[INFO] Simulation initialized.")

    def setup_environment(self):
        """Sets up the ground and robot."""
        self.create_ground()
        self.robot_id = self.create_robot()
        self.spawn_balloons(5)

    def create_ground(self):
        """Creates a simple ground plane."""
        p.loadURDF("plane.urdf")
        print("[INFO] Ground created.")

    def create_robot(self):
        """Creates a simple robot."""
        robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.2], useFixedBase=False)
        print("[INFO] Robot created at (0,0,0.2)")
        return robot_id

    def spawn_balloons(self, count):
        """Spawns a given number of balloons at random positions."""
        for _ in range(count):
            position = [random.uniform(-1, 1), random.uniform(-1, 1), 0.3]
            balloon_id = self.create_balloon(position)
            self.balloons.append(balloon_id)

    def create_balloon(self, position):
        """Creates a balloon (sphere) at a given position."""
        balloon_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
        balloon_collision = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
        balloon_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=balloon_collision,
            baseVisualShapeIndex=balloon_visual,
            basePosition=position
        )
        print(f"[INFO] Balloon created at {position}")
        return balloon_id

    def set_robot_position(self, position):
        """Sets the robot to a specific position."""
        p.resetBasePositionAndOrientation(self.robot_id, position, [0, 0, 0, 1])

    def check_collision(self, balloon_id):
        """Checks if the robot has popped a balloon."""
        contact_points = p.getContactPoints(self.robot_id, balloon_id)
        return len(contact_points) > 0

    def step_simulation(self):
        """Steps the simulation forward by one step."""
        p.stepSimulation()

    def run_simulation(self, steps=1000):
        """Runs the simulation loop without controlling the robot."""
        print("[INFO] Running simulation...")
        for step in range(steps):
            p.stepSimulation()
            time.sleep(1./60.)  # Slow down for visualization
        print("[INFO] Simulation completed.")
        p.disconnect()

    def cleanup(self):
        """Disconnects from the simulation."""
        p.disconnect()
        print("[INFO] Simulation cleaned up.")

if __name__ == "__main__":
    sim = BalloonSimulation()
    sim.setup_environment()
    sim.run_simulation()
