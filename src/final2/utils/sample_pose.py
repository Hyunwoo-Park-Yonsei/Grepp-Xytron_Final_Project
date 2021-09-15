import pickle

if __name__ == '__main__':
    file_name = "2021-09-14-pose_2"
    with open(file_name + ".pkl", "rb") as f:
        pose = pickle.load(f)

    sampled_pose = {'x': [], 'y': [], 'yaw': []}
    start = 1300
    last = len(pose['x'])
    steps = last // 300
    for i in range(start, len(pose['x'][:last]), steps):
        sampled_pose["x"].append(pose["x"][i])
        sampled_pose["y"].append(pose["y"][i])
        sampled_pose["yaw"].append(pose["yaw"][i])

    with open(file_name + "-sampled-300.pkl", "wb") as f:
        pickle.dump(sampled_pose, f)
