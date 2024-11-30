import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors
import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        # Read map size and costmap
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))
        
        costmap = []
        for line in file:
            if line.strip() == 'S':
                break
            # Split by comma and filter out empty strings
            values = [x for x in line.strip().split(',') if x]
            row = list(map(float, values))
            costmap.append(row)
        
        # Read all S and G positions
        positions = []
        while True:
            # Read start positions
            line = file.readline().strip()
            if not line:
                break
            coords = list(map(int, line.split(',')))
            start_pos = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
            
            # Read G marker
            assert file.readline().strip() == 'G', "Expected 'G' marker"
            
            # Read goal positions
            line = file.readline().strip()
            coords = list(map(int, line.split(',')))
            goal_pos = [(coords[i], coords[i+1]) for i in range(0, len(coords), 2)]
            
            positions.append((start_pos, goal_pos))
            
            # Read next S marker or EOF
            line = file.readline()
            if not line or line.strip() != 'S':
                break
        
        costmap = np.asarray(costmap)
        costmap = np.flipud(costmap)
    
    return x_size, y_size, costmap, positions

def update(frame, ax, positions, start_scatter, goal_scatter):
    start_pos, goal_pos = positions[frame]
    
    # Update scatter plots
    start_x, start_y = zip(*start_pos)
    goal_x, goal_y = zip(*goal_pos)
    
    start_scatter.set_offsets(np.c_[start_x, start_y])
    goal_scatter.set_offsets(np.c_[goal_x, goal_y])
    
    return start_scatter, goal_scatter

def save_animation(filename, positions, x_size, y_size, costmap):
    boundaries = np.arange(0, 1, 0.1)
    colors = plt.cm.plasma(np.linspace(0, 1, len(boundaries) - 1))
    cmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(boundaries, cmap.N, clip=True)
    
    fig, ax = plt.subplots()
    ax.imshow(costmap, cmap=cmap, norm=norm)
    start_scatter = ax.scatter([], [], c='g', marker='o', s=100, label='Start')
    goal_scatter = ax.scatter([], [], c='r', marker='o', s=100, label='Goal')
    ax.legend()
    
    anim = FuncAnimation(fig, update, frames=len(positions), 
                        fargs=(ax, positions, start_scatter, goal_scatter), 
                        interval=1, blit=True)
    anim.save(filename, writer='pillow')
    plt.close(fig)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)
    
    x_size, y_size, costmap, positions = parse_mapfile(sys.argv[1])

    # Define boundaries and colormap
    boundaries = np.arange(0, 1, 0.1)
    colors = plt.cm.plasma(np.linspace(0, 1, len(boundaries) - 1))
    cmap = mcolors.ListedColormap(colors)
    norm = mcolors.BoundaryNorm(boundaries, cmap.N, clip=True)

    # Create figure and axis
    fig, ax = plt.subplots()
    
    # Plot costmap
    cax = ax.imshow(costmap, cmap=cmap, norm=norm)
    
    # Initialize scatter plots for start and goal positions
    start_scatter = ax.scatter([], [], c='g', marker='o', s=100, label='Start')
    goal_scatter = ax.scatter([], [], c='r', marker='o', s=100, label='Goal')
    
    # Add colorbar and legend
    cbar = fig.colorbar(cax, ax=ax, ticks=[boundaries[0], boundaries[-1]])
    cbar.ax.set_yticklabels([f'{boundaries[0]}', f'{boundaries[-1]}'])
    ax.legend()
    
    # Create animation
    anim = FuncAnimation(
        fig, update, frames=len(positions),
        fargs=(ax, positions, start_scatter, goal_scatter),
        interval=200, blit=True
    )
    
    plt.show()
    # save_animation('output_animation.gif', positions, x_size, y_size, costmap)