import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from tracikpy import TracIKSolver
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import os, json

def pose_matrix(x, y, z, rpy):
    roll, pitch, yaw = rpy
    Rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    Ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = [x, y, z]
    return T

def quat_to_rot(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1-2*(yy+zz), 2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),     1-2*(xx+yy)]
    ])

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory,
                                              '/joint_trajectory_controller/joint_trajectory', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IK solver
        self.ik_solver = TracIKSolver(
            "/home/mscrobotics2425laptop36/diss3/gen3.urdf",
            "base_link",
            "tool_frame"
        )
        self.ik_joint_names = list(self.ik_solver.joint_names)

        # Seed persistence
        self.declare_parameter("persist_seed", True)
        self.persist_seed = self.get_parameter("persist_seed").value
        self.loaded_seed = self._load_seed()

        # Cache controller joint order
        self.controller_joint_names = None
        self.create_subscription(JointTrajectoryControllerState,
                                 '/joint_trajectory_controller/state',
                                 self._controller_state_cb,
                                 10)

        # Simple demo: publish once on timer
        self.timer = self.create_timer(5.0, self.demo_publish)
        self.sent = False

    # ------------------- Helpers -------------------
    def _seed_path(self):
        cfg_dir = os.path.expanduser("~/.config/spiral_tracik_executor")
        os.makedirs(cfg_dir, exist_ok=True)
        return os.path.join(cfg_dir, "seed.json")

    def _load_seed(self):
        try:
            with open(self._seed_path(), "r") as f:
                data = json.load(f)
            q = np.array(data.get("q", []), dtype=float)
            if q.size == self.ik_solver.number_of_joints:
                self.get_logger().info("Loaded persisted IK seed.")
                return q
        except Exception:
            pass
        return None

    def _save_seed(self, q):
        try:
            with open(self._seed_path(), "w") as f:
                json.dump({"q": list(map(float, q))}, f, indent=2)
            self.get_logger().info("Saved IK seed.")
        except Exception as e:
            self.get_logger().warn(f"Failed to save IK seed: {e}")

    def _controller_state_cb(self, msg: JointTrajectoryControllerState):
        if not self.controller_joint_names and msg.joint_names:
            self.controller_joint_names = list(msg.joint_names)
            self.get_logger().info(f"Controller joint order: {self.controller_joint_names}")

    def wait_for_tf(self, target, source, tries=50, dt=0.05):
        for _ in range(tries):
            try:
                return self.tf_buffer.lookup_transform(target, source, Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=dt)
        return None

    def world_to_base(self, p_world):
        tf = self.wait_for_tf('base_link', 'world')
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        tvec = np.array([t.x, t.y, t.z])
        return R @ p_world + tvec

    # ------------------- IK -------------------
    def ik_at(self, xyz_base, rpy, qseed):
        T = pose_matrix(*xyz_base, rpy=rpy)
        try:
            q = self.ik_solver.ik(T, qinit=qseed)
            return q
        except Exception as e:
            self.get_logger().warn(f"TRAC-IK exception at {xyz_base}: {e}")
            return None

    def nearest_reachable_on_plane_world(self, target_w, anchor_w, rpy, qseed, steps=24, tol=1e-4):
        p_base = self.world_to_base(np.array(target_w))
        if p_base is not None:
            q = self.ik_at(p_base, rpy, qseed)
            if q is not None:
                return q, p_base

        low, high = 0.0, 1.0
        for _ in range(steps):
            mid = 0.5 * (low + high)
            cand_w = (1 - mid) * np.array(target_w) + mid * np.array(anchor_w)
            cand_base = self.world_to_base(cand_w)
            if cand_base is None:
                break
            q_mid = self.ik_at(cand_base, rpy, qseed)
            if q_mid is not None:
                high = mid
                return q_mid, cand_base
            else:
                low = mid
            if high - low < tol:
                break
        return None, None

    def compute_joint_trajectory(self, cartesian_points, rpy=(np.pi,0,0)):
        traj = JointTrajectory()
        traj.joint_names = self.ik_joint_names

        q_prev = self.loaded_seed if self.loaded_seed is not None else np.zeros(self.ik_solver.number_of_joints)
        time_from_start = 0.0
        reached = 0

        anchor = cartesian_points[0]
        prev_reached = None

        for i, pw in enumerate(cartesian_points):
            p_base = self.world_to_base(np.array(pw))
            if p_base is None:
                continue

            q_sol = self.ik_at(p_base, rpy, q_prev)
            if q_sol is None:
                anchor_w = prev_reached if prev_reached is not None else anchor
                q_sol, used_p_base = self.nearest_reachable_on_plane_world(pw, anchor_w, rpy, q_prev)
            if q_sol is None:
                self.get_logger().warn(f"[{i}] Unreachable: {pw}")
                continue

            pt = JointTrajectoryPoint()
            pt.positions = list(q_sol)
            time_from_start += 0.3
            pt.time_from_start.sec = int(time_from_start)
            pt.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            traj.points.append(pt)

            q_prev = q_sol
            reached += 1
            if self.persist_seed and reached == 1:
                self._save_seed(q_sol)
            prev_reached = pw

        self.get_logger().info(f"IK succeeded for {reached}/{len(cartesian_points)} points")
        return self._remap_traj_if_needed(traj)

    def _remap_traj_if_needed(self, traj: JointTrajectory) -> JointTrajectory:
        if not self.controller_joint_names:
            return traj
        ctrl = self.controller_joint_names
        ik = self.ik_joint_names
        if ctrl == ik:
            return traj
        try:
            idx_map = [ik.index(name) for name in ctrl]
        except ValueError as e:
            self.get_logger().warn(f"Joint mismatch: {e}")
            return traj
        remapped = JointTrajectory()
        remapped.joint_names = list(ctrl)
        remapped.header = traj.header
        for p in traj.points:
            q_remap = [p.positions[j] for j in idx_map]
            pt = JointTrajectoryPoint()
            pt.positions = q_remap
            pt.time_from_start = p.time_from_start
            remapped.points.append(pt)
        self.get_logger().info("Remapped trajectory to controller order.")
        return remapped

    # ------------------- Demo -------------------
    def demo_publish(self):
        if self.sent:
            return
        # simple square in world frame
        square = [
            [0.3, 0.0, 0.4],
            [0.3, 0.1, 0.4],
            [0.4, 0.1, 0.4],
            [0.4, 0.0, 0.4]
        ]
        traj = self.compute_joint_trajectory(square)
        traj.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        self.traj_pub.publish(traj)
        self.get_logger().info("Published demo trajectory.")
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

