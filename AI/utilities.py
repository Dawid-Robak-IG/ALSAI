import tf_transformations

def quaternion_to_yaw(q):
    _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def get_pose_for_scan(scan_time, poses):
    left, right = 0, len(poses) - 1
    best_idx = 0
    min_diff = abs(scan_time - poses[0][0])
    
    while left <= right:
        mid = (left + right) // 2
        diff = abs(scan_time - poses[mid][0])
        if diff < min_diff:
            min_diff = diff
            best_idx = mid
        if poses[mid][0] < scan_time:
            left = mid + 1
        else:
            right = mid - 1
    
    return poses[best_idx][1]