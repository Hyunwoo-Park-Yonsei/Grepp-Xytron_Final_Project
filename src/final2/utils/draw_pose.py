import pickle
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__ == '__main__':
    file_name = "2021-09-14-pose_2-sampled-300"
    with open(file_name + ".pkl", "rb") as f:
        pose = pickle.load(f)

    start = 0
    last = len(pose['x'])
    steps = last // last
    plt.plot(pose['x'][start:last:steps], pose['y'][start:last:steps], '.', ms=3)
    for i in range(start, len(pose['x'][:last]), steps):
        plt.text(pose['x'][i], pose['y'][i], i, fontsize=3)
    plt.axis("equal")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.savefig(file_name, dpi=300)
