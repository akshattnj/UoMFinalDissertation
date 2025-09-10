import numpy as np
from tracikpy import TracIKSolver

# RViz-derived end-effector pose
ee_pose = np.array([
    [ 0, -0,  0.057,  0.070],
    [ 0,  0,  0.449, -0.313],
    [-0, -0,  0.891,  1.044],
    [ 0.000,  0.000,  0.000,  1.000]
])

# TracIK solver instantiation
ik_solver = TracIKSolver(
    "gen3.urdf",  # Replace with actual path
    "base_link",                      # Confirm from your URDF
    "tool_frame"                      # Confirm from your URDF
)

# Optional: print info
print("Number of joints:", ik_solver.number_of_joints)
print("Joint names:", ik_solver.joint_names)

# Solve IK
qout = ik_solver.ik(ee_pose, qinit=np.zeros(ik_solver.number_of_joints))

# Print result
if qout is not None:
    print("IK solution found:")
    print(qout)

    # Optional: verify with FK
    ee_out = ik_solver.fk(qout)
    ee_diff = np.linalg.inv(ee_pose) @ ee_out
    trans_err = np.linalg.norm(ee_diff[:3, 3], ord=1)
    angle_err = np.arccos(np.clip((np.trace(ee_diff[:3, :3]) - 1) / 2, -1.0, 1.0))

    print(f"Translation error: {trans_err:.6f}")
    print(f"Angular error (rad): {angle_err:.6f}")

    assert trans_err < 1e-3
    assert angle_err < 1e-3 or abs(angle_err - np.pi) < 1e-3
else:
    print("IK failed to find a solution.")
