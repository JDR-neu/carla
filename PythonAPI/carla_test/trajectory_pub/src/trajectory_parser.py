import re
import os
import sys

def loaddata(filepath, sample = 1):
    traj = []
    count = 0
    with open(filepath, 'r') as f:
        lines = f.readlines()
        for line in lines:
            count += 1
            if count == sample:
                content = re.findall(r'[(](.*?)[)]', line)
                xyz = content[0].split(', ')
                # print(xyz)
                x = xyz[0][2:]
                y = xyz[1][2:]
                z = xyz[2][2:]
                # print(type(x))
                # print(x, y, z)
                traj.append([float(x), float(y), float(z)])
                count = 0
    return traj


if __name__ == '__main__':
    path = os.path.abspath(os.path.dirname(sys.argv[0]))
    filepath = path + "/waypoint.txt"
    loaddata(filepath)