from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, PoseWithCovarianceStamped
import PyKDL
import tf2_ros

def to_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_to_msg(Vector3Stamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PoseStamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(PointStamped, to_msg_msg)

def from_msg_msg(msg):
    return msg

tf2_ros.ConvertRegistration().add_from_msg(Vector3Stamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PoseStamped, from_msg_msg)
tf2_ros.ConvertRegistration().add_from_msg(PointStamped, from_msg_msg)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))


# PointStamped
def do_transform_point(point, transform):
    p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
    res = PointStamped()
    res.point.x = p[0]
    res.point.y = p[1]
    res.point.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)


# Vector3Stamped
def do_transform_vector3(vector3, transform):
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    p = transform_to_kdl(transform) * PyKDL.Vector(vector3.vector.x, vector3.vector.y, vector3.vector.z)
    res = Vector3Stamped()
    res.vector.x = p[0]
    res.vector.y = p[1]
    res.vector.z = p[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(Vector3Stamped, do_transform_vector3)


# PoseStamped
def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PoseStamped, do_transform_pose)


# PoseWithCovarianceStamped
def do_transform_pose_with_covariance_stamped(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                                                                          pose.pose.pose.orientation.z, pose.pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z))
    res = PoseWithCovarianceStamped()
    res.pose.pose.position.x = f.p[0]
    res.pose.pose.position.y = f.p[1]
    res.pose.pose.position.z = f.p[2]
    (res.pose.pose.orientation.x, res.pose.pose.orientation.y, res.pose.pose.orientation.z, res.pose.pose.orientation.w) = f.M.GetQuaternion()
    res.pose.covariance = pose.pose.covariance
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(PoseWithCovarianceStamped, do_transform_pose_with_covariance_stamped)