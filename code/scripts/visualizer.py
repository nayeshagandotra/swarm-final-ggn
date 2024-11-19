import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation

import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))

        line = file.readline().strip()

        costmap = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)
        
        costmap = np.asarray(costmap).T
    
    return x_size, y_size, costmap

SPEEDUP = 5000

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)
    
    x_size, y_size, costmap = parse_mapfile(sys.argv[1])

    fig, ax = plt.subplots()
    
    ax.imshow(costmap)

    plt.legend()
    plt.show()
