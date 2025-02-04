import pybullet as p
import pybullet_data
import time
import random
import numpy as np

def create_ground():
    """Creates a simple ground plane."""
    p.loadURDF("plane.urdf")

def create_robot():
    """Creates a simple robot with an arm."""
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.2], useFixedBase=False)
    print("[INFO] Robot created at (0,0,0.2)")
    return robot_id

def create_balloon(position):
    """Creates a balloon (sphere) at a given position."""
    balloon_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
    balloon_collision = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
    balloon_id = p.createMultiBody(baseMass=0.05, baseCollisionShapeIndex=balloon_collision,
                                   baseVisualShapeIndex=balloon_visual, basePosition=position)
    print(f"[INFO] Balloon created at {position}")
    return balloon_id

def move_robot(robot_id, target_pos):
    """Moves the robot towards a target position."""
    current_pos, _ = p.getBasePositionAndOrientation(robot_id)
    direction = np.array(target_pos) - np.array(current_pos)
    direction /= np.linalg.norm(direction) + 1e-6  # Normalize vector
    new_pos = np.array(current_pos) + 0.02 * direction  # Small step movement
    p.resetBasePositionAndOrientation(robot_id, new_pos.tolist(), [0, 0, 0, 1])

    # Visualize movement trail
    p.addUserDebugLine(current_pos, new_pos.tolist(), [0, 1, 0], 2, 0.2)

def check_collision(robot_id, balloon_id):
    """Checks if the robot has popped a balloon."""
    contact_points = p.getContactPoints(robot_id, balloon_id)
    return len(contact_points) > 0

def main():
    """Main simulation loop with enhanced visualizations and logging."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    print("[INFO] Simulation started")
    create_ground()
    robot_id = create_robot()

    balloons = [create_balloon([random.uniform(-1, 1), random.uniform(-1, 1), 0.3]) for _ in range(5)]
    popped_count = 0

    for step in range(1000):  # Simulation steps
        if not balloons:
            print("[INFO] All balloons popped! Ending simulation.")
            break

        for balloon_id in balloons[:]:  # Iterate over a copy to modify the list
            balloon_pos = p.getBasePositionAndOrientation(balloon_id)[0]

            for _ in range(1000):
                move_robot(robot_id, balloon_pos)

                if check_collision(robot_id, balloon_id):

                    p.removeBody(balloon_id)
                    balloons.remove(balloon_id)
                    popped_count += 1
                    print(f"[POP] Balloon at {balloon_pos} popped!")
                    p.stepSimulation()
                    break

                p.stepSimulation()
                time.sleep(1./60.)  # Slow down for visualization

    print(f"[INFO] Simulation ended. Total balloons popped: {popped_count}")
    p.disconnect()

if __name__ == "__main__":
    main()