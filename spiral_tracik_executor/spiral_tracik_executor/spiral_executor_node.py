import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tracikpy import TracIKSolver
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from PIL import Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration  # ADD
from control_msgs.msg import JointTrajectoryControllerState  # ADD
import cv2
from deepskin import wound_segmentation
from deepskin.imgproc import imfill, get_perilesion_mask
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
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
    # Standard quaternion -> rotation matrix
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return np.array([
        [1-2*(yy+zz), 2*(xy-wz),     2*(xz+wy)],
        [2*(xy+wz),   1-2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),   2*(yz+wx),     1-2*(xx+yy)]
    ])

class SpiralExecutor(Node):
    def __init__(self):
        super().__init__('spiral_executor')

        # Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/spiral_markers', 10)
        self.image_marker_pub = self.create_publisher(Marker, '/wound_image_marker', 1)
        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')


        # TF2 (for world -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # IK solver (kept exactly as you had it)
        self.ik_solver = TracIKSolver(
            "/home/mscrobotics2425laptop36/diss3/gen3.urdf",
            "base_link",
            "tool_frame"
        )
        # OPTIONAL (commented): bias TRAC-IK to stay close to seed
        # self.ik_solver = TracIKSolver(
        #     "/home/mscrobotics2425laptop36/diss3/gen3.urdf",
        #     "base_link",
        #     "tool_frame",
        #     solve_type="Distance"
        # )

        # Config: same spiral numbers, but in WORLD frame at constant Z = "table height"
        self.plane_frame = "base_link"  # or "world" if you prefer
        self.table_height = 0.4
        self.center_world = np.array([0.3, 0.0, self.table_height])

        # ADD: IK joint order cache (also used by remapper)
        self.ik_joint_names = list(self.ik_solver.joint_names)

        # ADD: seed persistence controls
        self.declare_parameter("persist_seed", True)
        self.persist_seed = self.get_parameter("persist_seed").get_parameter_value().bool_value
        self.loaded_seed = self._load_seed()

        # self.traj = self.compute_joint_trajectory()
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        self.sent = False  # publish-once guard

        # ADD: capture controller joint order from its state topic
        self.controller_joint_names = None
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self._controller_state_cb,
            10
        )

        # ADD: home sequencing
        self.home_sent = False
        self.home_done = False
        self.home_duration_s = 3.0
        self.home_timer = self.create_timer(0.5, self.publish_home)
        self._home_start_time = None
        self._home_check_timer = None

    def publish_spiral_markers(self, points_world):
        markers = MarkerArray()
        for i, (x, y, z) in enumerate(points_world):
            m = Marker()
            m.header.frame_id = self.plane_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "spiral"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.008
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    # ADD: simple seed persistence helpers
    def _seed_path(self):
        cfg_dir = os.path.expanduser("~/.config/spiral_tracik_executor")
        os.makedirs(cfg_dir, exist_ok=True)
        return os.path.join(cfg_dir, "seed.json")
    
    def send_gripper_goal(self, position: float, max_effort: float = 50.0):
        """Send a position goal to the Robotiq gripper."""
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Gripper action server not available.")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)   # e.g. 0.35 open, 0.0 closed
        goal_msg.command.max_effort = float(max_effort)

        self.gripper_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent gripper goal: position={position:.3f}, effort={max_effort}")


    
    def publish_image_plane(self, image_path, frame="base_link"):
        marker = Marker()
        marker.header.frame_id = self.plane_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wound_image"
        marker.id = 999
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.center_world[0])
        marker.pose.position.y = float(self.center_world[1])
        marker.pose.position.z = float(self.center_world[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08  # width of the image
        marker.scale.y = 0.08  # height of the image
        marker.scale.z = 0.001  # paper-thin

        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.image_marker_pub.publish(marker)

        self.get_logger().info(f"Published wound image plane at {self.center_world} with frame {frame}.")



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
            self.get_logger().info("Saved IK seed for future runs.")
        except Exception as e:
            self.get_logger().warn(f"Failed to save IK seed: {e}")

    # ADD:
    def _controller_state_cb(self, msg: JointTrajectoryControllerState):
        if not self.controller_joint_names and msg.joint_names:
            self.controller_joint_names = list(msg.joint_names)
            self.get_logger().info(f"Controller joint order: {self.controller_joint_names}")

    def wait_for_tf(self, target, source, tries=50, dt=0.05):
        for _ in range(tries):
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time())
                return tf
            except Exception:
                rclpy.spin_once(self, timeout_sec=dt)
        return None

    def world_to_base(self, p_world):
        tf = self.wait_for_tf('base_link', self.plane_frame)
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)  # maps world -> base_link
        tvec = np.array([t.x, t.y, t.z])
        return R @ p_world + tvec

    # def generate_spiral_world(self):
    #     spiral_world = []
    #     for t in np.linspace(0, 4 * np.pi, 50):
    #         spiral_radius_mm = 30  # 3 cm wound
    #         spiral_world = []
    #         for t in np.linspace(0, 4 * np.pi, 50):
    #             r = (spiral_radius_mm / 1000.0) * (t / (4 * np.pi))  # gradual from 0 to max radius
    #             xw = self.center_world[0] + r * np.cos(t)
    #             yw = self.center_world[1] + r * np.sin(t)
    #             zw = self.table_height
    #             spiral_world.append((xw, yw, zw))        
    #     self.publish_spiral_markers(spiral_world)
    #     self.get_logger().info(f"Generated spiral world with {len(spiral_world)} points.")
    #     return spiral_world

    def generate_spiral_from_image(self):
        # image_path = "/home/mscrobotics2425laptop36/ABRASION.jpg"
        global image_path
        image_path = os.path.expanduser("~/.ros/rviz_textures/abrasion.png")
        if not os.path.exists(image_path):
            self.get_logger().warn("Image not found at ABRASION.jpg")
            return []

        try:
            img = cv2.imread(image_path)[..., ::-1]  # BGR to RGB
            if img is None:
                self.get_logger().warn("Failed to load image.")
                return []

            # DeepSkin segmentation
            seg = wound_segmentation(img=img, tol=0.95, verbose=False)
            wound_mask, body_mask, _ = cv2.split(seg)

            wound_mask = (wound_mask > 0).astype(np.uint8)
            body_mask = (body_mask > 0).astype(np.uint8)

            peri_mask = get_perilesion_mask(mask=wound_mask * 255, ksize=(200, 200))
            peri_mask = cv2.bitwise_and(peri_mask, peri_mask, mask=imfill((body_mask | wound_mask) * 255))

            M = cv2.moments(wound_mask)
            cx = int(M["m10"] / (M["m00"] + 1e-5))
            cy = int(M["m01"] / (M["m00"] + 1e-5))

            contours, _ = cv2.findContours(peri_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_r = max(np.sqrt((pt[0][0] - cx)**2 + (pt[0][1] - cy)**2) for cnt in contours for pt in cnt)

            px_to_m = 0.0001  # 0.1 mm/px
            r_m = max_r * px_to_m

            spiral = []
            for t in np.linspace(0, 4 * np.pi, 50):
                scaled_r = r_m * (t / (4 * np.pi))
                dx = scaled_r * np.cos(t)
                dy = scaled_r * np.sin(t)
                xw = self.center_world[0] + dx
                yw = self.center_world[1] + dy
                zw = self.table_height + 0.0983 + 0.08
                spiral.append((xw, yw, zw))

            self.publish_spiral_markers(spiral)
            self.publish_image_plane(image_path, frame=self.plane_frame)
            return spiral

        except Exception as e:
            self.get_logger().warn(f"DeepSkin error: {e}")
            return []


    def ik_at(self, xyz_base, rpy, qseed):
        T = pose_matrix(*xyz_base, rpy=rpy)
        try:
            q = self.ik_solver.ik(T, qinit=qseed)
        except Exception as e:
            self.get_logger().warn(f"TRAC-IK exception at {xyz_base}: {e}")
            q = None
        return (q, xyz_base) if q is not None else (None, None)

    def nearest_reachable_on_plane_world(self, target_w, anchor_w, rpy, qseed, steps=24, tol=1e-4):
        # Try target first
        p_base = self.world_to_base(np.array(target_w))
        if p_base is not None:
            q, _ = self.ik_at(p_base, rpy, qseed)
            if q is not None:
                return q, p_base, 0.0

        low, high = 0.0, 1.0
        best = (None, None, None)
        for _ in range(steps):
            mid = 0.5 * (low + high)
            cand_w = (1 - mid) * np.array(target_w) + mid * np.array(anchor_w)
            cand_base = self.world_to_base(cand_w)
            if cand_base is None:
                break
            q_mid, _ = self.ik_at(cand_base, rpy, qseed)
            if q_mid is not None:
                best = (q_mid, cand_base, mid)
                high = mid
            else:
                low = mid
            if high - low < tol:
                break
        return best

    def compute_joint_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = list(self.ik_solver.joint_names)

        tf_ok = self.wait_for_tf('base_link', self.plane_frame) is not None
        if not tf_ok:
            self.get_logger().warn("TF world->base_link not available yet. "
                                   "Falling back to base_link plane (may look vertical if frames differ).")

        spiral_world = self.generate_spiral_from_image()

        fixed_rpy = (np.pi, 0.0, 0.0)  # tool down
        q_prev = np.zeros(self.ik_solver.number_of_joints)
        # ADD: use persisted seed if available, otherwise keep zeros
        if self.loaded_seed is not None:
            q_prev = self.loaded_seed.copy()

        time_from_start = 0.0
        reached = 0

        anchor_world = spiral_world[0]
        prev_world_reached = None

        for i, pw in enumerate(spiral_world):
            p_base = self.world_to_base(np.array(pw)) if tf_ok else np.array([pw[0], pw[1], pw[2]])
            q_sol, used_p_base = self.ik_at(p_base, fixed_rpy, q_prev) if p_base is not None else (None, None)

            if q_sol is None:
                line_anchor_world = prev_world_reached if prev_world_reached is not None else anchor_world
                q_sol, used_p_base, t_found = self.nearest_reachable_on_plane_world(
                    pw, line_anchor_world, fixed_rpy, q_prev
                )

            if q_sol is None:
                self.get_logger().warn(f"[{i}] Unreachable near world point {pw}. Skipping.")
                continue

            # Add trajectory point
            pt = JointTrajectoryPoint()
            pt.positions = list(q_sol)
            time_from_start += 0.3
            pt.time_from_start.sec = int(time_from_start)
            pt.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            traj.points.append(pt)

            q_prev = q_sol
            reached += 1

            # ADD: persist the very first successful IK solution as seed (for next runs)
            if self.persist_seed and reached == 1:
                self._save_seed(q_sol)

            prev_world_reached = pw
            self.get_logger().info(f"[{i}] IK OK (world {pw})")

        self.get_logger().info(f"IK succeeded for {reached} / {len(spiral_world)} points.")
        return traj

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
            self.get_logger().warn(f"Controller has joints not in IK list: {e}. Using original order.")
            return traj
        remapped = JointTrajectory()
        remapped.joint_names = list(ctrl)
        remapped.header = traj.header
        for p in traj.points:
            q = list(p.positions)
            q_remap = [q[j] for j in idx_map]
            pt = JointTrajectoryPoint()
            pt.positions = q_remap
            pt.time_from_start = p.time_from_start
            remapped.points.append(pt)
        self.get_logger().info("Remapped trajectory to controller joint order.")
        return remapped
    
    def publish_home(self):
        """Send the arm to home (all zeros) once, then mark home as done after duration."""
        if self.home_sent:
            return

        # Ensure someone is listening
        wait_tries = 50
        while self.traj_pub.get_subscription_count() == 0 and wait_tries > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_tries -= 1
        if self.traj_pub.get_subscription_count() == 0:
            self.get_logger().warn("No subscribers on /joint_trajectory_controller/joint_trajectory; not sending home yet.")
            return

        home = JointTrajectory()
        home.joint_names = list(self.ik_joint_names)  # IK order; remapper fixes if needed
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * len(home.joint_names)
        pt.time_from_start.sec = int(self.home_duration_s)
        pt.time_from_start.nanosec = int((self.home_duration_s % 1.0) * 1e9)
        home.points.append(pt)

        # Start relative to receipt time
        home.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()

        home_to_send = self._remap_traj_if_needed(home)
        self.traj_pub.publish(home_to_send)
        self.get_logger().info("Published home trajectory (all zeros).")

        self.home_sent = True
        self._home_start_time = self.get_clock().now()
        if self._home_check_timer is None:
            self._home_check_timer = self.create_timer(0.2, self._check_home_complete)
        self.home_timer.cancel()

    def _check_home_complete(self):
        if self._home_start_time is None:
            return
        if (self.get_clock().now() - self._home_start_time) >= Duration(seconds=self.home_duration_s):
            self.home_done = True
            self.get_logger().info("Home motion presumed complete.")
            if self._home_check_timer is not None:
                self._home_check_timer.cancel()
                self._home_check_timer = None

    def publish_gripper_trajectory(self, num_points, total_time, open_pos=0.1, closed_pos=0.7):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Gripper action server not available.")
            return

        for i in range(num_points):
            frac = i / max(1, num_points - 1)   # goes 0.0 -> 1.0
            pos = open_pos + frac * (closed_pos - open_pos)

            t = i * (total_time / num_points)
            self.create_timer(t, lambda p=pos: self.send_gripper_goal(position=p))

    def publish_trajectory(self):
        # Don't publish spiral until home is done
        # Force open at the start
        self.send_gripper_goal(position=0.1)   # fully open

        if not getattr(self, 'home_done', True):
            return

        # don't keep restarting the controller
        if self.sent:
            return

        # Wait for subscriber
        wait_tries = 50
        while self.traj_pub.get_subscription_count() == 0 and wait_tries > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_tries -= 1
        if self.traj_pub.get_subscription_count() == 0:
            self.get_logger().warn("No subscribers on /joint_trajectory_controller/joint_trajectory; not sending yet.")
            return

        # # Your original line (kept)
        # self.traj.header.stamp = self.get_clock().now().to_msg()
        # # Override with zero stamp so times are relative to receipt
        # self.traj.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        


        traj = self.compute_joint_trajectory()
        traj.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
        traj_to_send = self._remap_traj_if_needed(traj)


        self.traj_pub.publish(traj_to_send)
        self.get_logger().info("Published full spiral trajectory.")
        if self.gripper_client.wait_for_server(timeout_sec=2.0):
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = 0.7      # fully closed
            goal_msg.command.max_effort = 50.0
            self.gripper_client.send_goal_async(goal_msg)
            self.get_logger().info("Closing gripper during spiral (0.1 → 0.7).")
        else:
            self.get_logger().warn("Gripper action server not available.")


        self.get_logger().info("Joint state simulation complete.")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SpiralExecutor()
    rclpy.spin(node)
    rclpy.shutdown()
