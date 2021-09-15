import rosbag
import pickle
import tf

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

if __name__ == '__main__':
    bag = rosbag.Bag("/home/nvidia/xycar_ws/src/xycar_slam/bags/2021-09-14-pose+trajectory_2.bag")
    
    pose = {'x': [], 'y': [], 'yaw': []}
    traj = {'x': [], 'y': [], 'yaw': []}
    
    for topic, msg, time in bag.read_messages(topics=['/tracked_pose', '/trajectory_node_list']):
        if topic == '/tracked_pose':
            pose['x'].append(msg.pose.position.x)
            pose['y'].append(msg.pose.position.y)

            q_pose = msg.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([q_pose.x, q_pose.y, q_pose.z, q_pose.w])
            pose['yaw'].append(yaw)

        
    # with open("pose.pkl", "wb") as f_pose, open("trajectory.pkl", "wb") as f_traj:
    with open("pose.pkl", "wb") as f_pose:
        pickle.dump(pose, f_pose)
        # pickle.dump(traj, f_traj)


