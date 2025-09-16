import tf_transformations

def quaternion_to_yaw(q):
    _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw