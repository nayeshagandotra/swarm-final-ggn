import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors
import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))
        
        costmap = []
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)
        
        costmap = np.asarray(costmap)
        # Flip the array vertically to make bottom left corner (0,0)
        costmap = np.flipud(costmap)
    
    return x_size, y_size, costmap

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)
    
    x_size, y_size, costmap = parse_mapfile(sys.argv[1])

    # Define boundaries and colormap
    boundaries = np.arange(0, 70, 1)  # Boundaries from 0 to 500, incremented by 20
    colors = plt.cm.plasma(np.linspace(0, 1, len(boundaries) - 1))  # Generate a range of purples

    # Create a colormap from the defined colors
    cmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(boundaries, cmap.N, clip=True)

    # Create a plot
    fig, ax = plt.subplots()
    cax = ax.imshow(costmap, cmap=cmap, norm=norm)

    # Add colorbar with only the first and last ticks
    cbar = fig.colorbar(cax, ax=ax, ticks=[boundaries[0], boundaries[-1]])
    cbar.ax.set_yticklabels([f'{boundaries[0]}', f'{boundaries[-1]}'])  # Set labels for colorbar
    
    plt.show()