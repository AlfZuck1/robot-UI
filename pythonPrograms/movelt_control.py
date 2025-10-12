from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

class CartesianControlNode(ControlNode):
    def __init__(self):
        super().__init__()
        self.move_group = MoveGroupCommander("arm")  # nombre de tu grupo MoveIt
        logging.info("MoveGroupCommander ready for Cartesian control.")

    def move_to_cartesian_pose(self, x, y, z, roll, pitch, yaw):
        pose_target = Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        # Convertir RPY → cuaternión
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        self.move_group.set_pose_target(pose_target)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            logging.info(f"TCP moved to ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            logging.warning("No se pudo alcanzar la posición cartesiana solicitada.")
