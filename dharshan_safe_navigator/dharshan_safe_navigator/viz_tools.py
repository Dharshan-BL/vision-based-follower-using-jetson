

import numpy as np

from matplotlib.patches import PathPatch, Circle
from matplotlib import patches
from matplotlib import colors as mcolors
from matplotlib.patches import Polygon as pltPolygon

from matplotlib.path  import Path
from scipy.ndimage import gaussian_filter


def blend_colors(foreground, alpha, background=(1,1,1)):
    """
    Blend two colors with a given alpha value
    """
    foreground = mcolors.hex2color(foreground)
    blended_color = [alpha * foreground_channel + (1 - alpha) * background_channel
                     for foreground_channel, background_channel in zip(foreground, background)]
    return tuple(blended_color)

def plot_square_patch(ax, center_point, grid_size, voronoi_color):
    """
    Plots a square patch with a given center point, size, and color.

    Parameters:
    - center_point: A tuple (x, y) representing the center of the square.
    - grid_size: The size of the square.
    - voronoi_color: The fill color of the square.
    - edge_color: The edge color of the square.
    """
    # Calculate the corners of the square
    x = center_point.x
    y = center_point.y
    half_size = grid_size / 2
    vertices = [(x - half_size, y - half_size),
                (x + half_size, y - half_size),
                (x + half_size, y + half_size),
                (x - half_size, y + half_size)]

    # Create and add the square patch
    opaque_color = blend_colors(voronoi_color, 0.5)

    square = pltPolygon(vertices, closed=True, facecolor=opaque_color)
    ax.add_patch(square)


def plot_keyframe_icon(ax, center, radius, color, edge_color='black'):
    """
    Plots a filled circle with a large star marker that fits inside the circle.

    Parameters:
    - center: A tuple (x, y) representing the center of the circle.
    - radius: The radius of the circle.
    - color: The fill color of the circle and star.
    - edge_color: The edge color of the circle and star.
    """
    # Plot the circle
    circle = patches.Circle(center, radius, facecolor=color, edgecolor=edge_color, linewidth=1.5, zorder=5)
    ax.add_patch(circle)

    scaling_factor = 600  # Adjust this as needed to match the visual size
    star_size = np.pi * (radius ** 2) * scaling_factor

    # Plot the star
    ax.scatter(*center, color=color, s=star_size, marker='*', edgecolors='black', linewidths=1.0, zorder=5)


def plot_fancy_circular_robot(dia, x, y):
    # Create a figure and an axis
    # Define the robot's body (circle with a fancy border and pattern)
    body = patches.Circle((x, y), dia / 2, edgecolor='black', facecolor='gray', linewidth=1, zorder=3)
    pattern = patches.Circle((x, y), dia / 2 * 0.9, edgecolor='black', facecolor='none', linestyle='dotted', linewidth=1, zorder=3)

    # Add shapes to the plot
    # ax.add_patch(body)
    # ax.add_patch(pattern)

    # Add some fancy elements (like additional circles to make it look more technical)
    inner_circle = patches.Circle((x, y), dia / 2 * 0.4, edgecolor='black', facecolor='lightgrey', linewidth=1, zorder=3)
    # ax.add_patch(inner_circle)

    return body, pattern, inner_circle

def plot_unicycle_robot(dia, x, y, theta):
    
    body = patches.Circle((x, y), dia / 2, edgecolor='black', facecolor='gray', linewidth=1, zorder=3)

    # Heading direction as a rectangular patch
    slot_width = dia * 0.2
    slot_length = dia/2 
    
    # Calculate the rectangle's position to ensure it is centered and aligned with the robot's orientation
    slot_x = x + slot_width/2 * np.sin(theta)
    slot_y = y - slot_width/2 * np.cos(theta)

    slot = patches.Rectangle(
        (slot_x, slot_y),
        slot_length,
        slot_width,
        angle=np.degrees(theta),
        edgecolor='black',
        facecolor='lightgrey',
        linewidth=0.1,
        zorder=4
    )

    return body, slot
    




def voronoi_color_scheme(xlim, ylim, x, y):
    xmin, xmax = xlim
    ymin, ymax = ylim

    xmin = xmin + 1.0
    xmax = xmax - 1.0
    ymin = ymin + 1.0
    ymax = ymax - 1.0
    
    # Normalized coordinates
    tx = (x - xmin ) / (xmax - xmin)
    ty = (y - ymin ) / (ymax - ymin)
    
    # Colors at the corners [R, G, B, Alpha]
    # bottom_left = np.array([0.4660, 0.6740, 0.1880, 1.0])   # Green
    # bottom_right = np.array([0.4940, 0.1840, 0.5560, 1.0])        # Violet
    # top_left = np.array([0.9290, 0.6940, 0.1250, 1.0])      # Yellow
    # top_right = np.array([0.3010, 0.7450, 0.9330, 1.0])     # cyan

    bottom_left = np.array([110, 171, 71]) / 255 # Green
    top_left = np.array([221, 192, 30]) / 255 # Yellow
    top_right = np.array([90, 180, 213]) / 255 # Cyan
    bottom_right = np.array([177, 111, 227]) / 255 # Violet


    # Distances to each corner
    d_tl = np.sqrt(tx**2 + ty**2)
    d_tr = np.sqrt((1 - tx)**2 + ty**2)
    d_bl = np.sqrt(tx**2 + (1 - ty)**2)
    d_br = np.sqrt((1 - tx)**2 + (1 - ty)**2)
    
    # Avoid division by zero
    d_tl = max(d_tl, 1e-10)
    d_tr = max(d_tr, 1e-10)
    d_bl = max(d_bl, 1e-10)
    d_br = max(d_br, 1e-10)
    
    # Weights inversely proportional to the distances
    w_tl = 1 / d_tl
    w_tr = 1 / d_tr
    w_bl = 1 / d_bl
    w_br = 1 / d_br
    
    # Normalize the weights
    total_weight = w_tl + w_tr + w_bl + w_br
    w_tl /= total_weight
    w_tr /= total_weight
    w_bl /= total_weight
    w_br /= total_weight
    
    # Compute the final color
    final_color = w_tl * bottom_left + w_tr * bottom_right + w_bl * top_left + w_br * top_right
    
    return final_color


def plot_polygon_with_gradient(vertices, center, color, sigma=3, smooth=20):
    """
    Plots a polygon with a yellow gradient that is more opaque at the center and more transparent at the edges.
    
    Parameters:
    - vertices: A list of tuples/lists representing the vertices of the polygon [(x1, y1), (x2, y2), ...]
    - sigma: Controls the spread of the gradient. Higher values make the center more concentrated.
    - smooth: The smoothing factor for the gradient edges.
    """
    # Create a Path from the vertices
    codes = [Path.MOVETO] + [Path.LINETO] * (len(vertices) - 2) + [Path.CLOSEPOLY]
    path = Path(vertices, codes)
    
    # Extent based on vertices for better placement
    x_min, x_max = np.min(vertices, axis=0)[0], np.max(vertices, axis=0)[0]
    y_min, y_max = np.min(vertices, axis=0)[1], np.max(vertices, axis=0)[1]
    x = np.linspace(x_min, x_max, 500)
    y = np.linspace(y_min, y_max, 500)
    X, Y = np.meshgrid(x, y)

    # Calculate the distance from the center of the polygon
    center = np.array(center)
    R = np.sqrt((X - center[0])**2 + (Y - center[1])**2)

    # Apply an inverted Gaussian function as the alpha gradient
    alpha = np.exp(-(R**2 / (2.0 * sigma**2)))

    # Blur the alpha channel to make it smoother
    alpha = gaussian_filter(alpha, sigma=smooth)

    # Create RGBA values for yellow with varying alpha
    rgba = np.zeros((*alpha.shape, 4))
    rgba[..., 0] = color[0]  # Red channel
    rgba[..., 1] = color[1]  # Green channel
    rgba[..., 2] = color[2]  # Blue channel
    rgba[..., 3] = alpha  # Alpha channel

    # Display the RGBA image
    # im = self.ax1.imshow(rgba, extent=(x_min, x_max, y_min, y_max), origin='lower')

    # Create and add the polygon
    polygon = pltPolygon(vertices, closed=True, facecolor='none', edgecolor='r')
    # self.ax1.add_patch(polygon)

    # Clipping the image with the path using a transform
    # patch = PathPatch(path, facecolor='none', edgecolor='none')
    # self.ax1.add_patch(patch)
    # im.set_clip_path(patch)

