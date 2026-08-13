import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPolygon
from matplotlib.collections import PatchCollection
from matplotlib.path import Path as MplPath


def center_radius_generator_distribution(map_distance_meter,map_resolution,keep_out_zone,robot_size,k,b,Dmin,Dmax):
    """
    Create centerpoints and radii sampled from an uniform distribution.
    
    Args:
        domain_size: int value to compute the expectation value of the amount of craters
        k,b: cumulative law parameters, given by geoscience  N(>D) = k D^-b  (per m²)
        D_min : smallest diameter [m]
        D_max : largest diameter [m]
    
    Returns:
        List of touples of the center points, normalized between 0 and 1, List for the radii

    """

    crater_amount_expected = map_distance_meter**2 * k * Dmin**(-b)
    #np.random.seed(seed) not used
    crater_amount_poisson = np.random.poisson(crater_amount_expected)

    # Create an array of the given shape and populate it with random samples from a uniform distribution over [0, 1).
    crater_uniform = np.random.rand(crater_amount_poisson)

    D_m = (Dmin**(-b) - crater_uniform*(Dmin**(-b)-Dmax**(-b)))**(-1/b)

    # add the same diameter as visible for the disturbed ground (only needed for young crater) and the robot footprint
    R_m = D_m / 2 * (1 + keep_out_zone) + robot_size

    # create centers
    x_m = np.random.rand(crater_amount_poisson)
    y_m = np.random.rand(crater_amount_poisson)
    center = np.column_stack((x_m,y_m))

    return center, R_m

def rasterize_circles(center_radius, map_size, map_resolution):

    grid_size = int(round(map_resolution))
    cell_size = map_size / grid_size
    grid = np.zeros((grid_size, grid_size), dtype=int)

    x_px = center_radius["center"][:,0] * map_size
    y_px = center_radius["center"][:,1] * map_size

    for cx, cy, r in zip(x_px,y_px,center_radius["radius"]):

        # compute bounding box in grid coordinates
        min_x = max(int((cx - r) // cell_size), 0)
        max_x = min(int((cx + r) // cell_size), grid_size - 1)

        min_y = max(int((cy - r) // cell_size), 0)
        max_y = min(int((cy + r) // cell_size), grid_size - 1)

        for i in range(min_x, max_x + 1):
            for j in range(min_y, max_y + 1):

                cell_x = i * cell_size
                cell_y = j * cell_size

                if circle_intersects_cell(cx, cy, r, cell_x, cell_y, cell_size):
                    grid[j, i] = 1
    
    return grid

def rasterize_polygon(vertices_norm, map_size, map_resolution):
    """
    Rasterize a single closed polygon (e.g. the pit outline), given as
    normalized [0, 1] map-coordinate vertices, into the same binary grid
    convention as rasterize_circles (cell marked if its center falls
    inside the polygon).
    """
    grid_size = int(round(map_resolution))
    cell_size = map_size / grid_size

    poly_path = MplPath(vertices_norm * map_size)

    cell_centers = (np.arange(grid_size) + 0.5) * cell_size
    xx, yy = np.meshgrid(cell_centers, cell_centers)
    points = np.column_stack((xx.ravel(), yy.ravel()))

    inside = poly_path.contains_points(points).reshape(grid_size, grid_size)
    return inside.astype(int)


def downsample_grid_max(grid, factor):
    """
    Shrink a binary occupancy grid by max-pooling factor x factor blocks, so any
    obstacle pixel in a block marks the whole coarse cell as blocked. Used to get
    a much smaller/faster grid for path planning while keeping the full-resolution
    grid for visualization and physical stats (e.g. grid_occupation).
    """
    if factor <= 1:
        return grid

    h, w = grid.shape
    h_crop = (h // factor) * factor
    w_crop = (w // factor) * factor
    cropped = grid[:h_crop, :w_crop]

    reshaped = cropped.reshape(h_crop // factor, factor, w_crop // factor, factor)
    return reshaped.max(axis=(1, 3))

def distribution_vizualization2(map_distance_meter,map_resolution,data, ABOVE_THREASHOLD):
    
    # Scale the crator_center location to the map size
    centers = data["center"] * map_distance_meter 
    radii_meter = data["radius"]
    
    # for plotting circles
    x_px = centers[:,0]#data["center"][:,0] 
    y_px = centers[:,1]#data["center"][:,1] 

    # Plot in pixel coordinates
    fig, ax = plt.subplots(figsize=(10,10))
    #plt.imshow(image, cmap='binary', origin='lower')

    for xi, yi, ri, t in zip(x_px, y_px, radii_meter, data["type"]):
        edge_color = "blue" if t == "crater" else "red"
        # different alpha for larger craters
        alpha = 0.6 if (t == "crater" and ri > ABOVE_THREASHOLD) else 0
        circle = plt.Circle((xi, yi), ri, fill=True, edgecolor=edge_color, alpha=alpha)
        # thicker outline for large craters
        #linewidth = 2.5 if (t == "crater" and ri > ABOVE_THREASHOLD) else 1.0
        #circle = plt.Circle((xi, yi), ri, fill=False, edgecolor=edge_color, linewidth = linewidth, alpha=0.9)
        ax.add_patch(circle)

    ax.set_xlim(0,map_distance_meter)
    ax.set_ylim(0,map_distance_meter)
    ax.legend(f"Resolution {map_resolution} m per Pixel")
    ax.set_xlabel("Meter")
    ax.set_ylabel("Meter")
    ax.set_aspect('equal')
    ax.set_title("Map of Lunar Surface")
    plt.figtext(0.1,0.9,f"Map Size is {map_distance_meter} meter")
    legend_large_crater = plt.Circle((0,0), 1, edgecolor='blue', fill=True, alpha=0.6, label='Crater (above resolution)')
    legend_small_crater = plt.Circle((0,0), 1, edgecolor='blue', fill=True, alpha=0.2, label='Crater (below resolution)')
    legend_other = plt.Circle((0,0), 1, edgecolor='red', fill=True, alpha=0.2, label='Boulders')
    ax.legend(handles=[legend_small_crater, legend_large_crater, legend_other], loc="upper right")

    return ax

    
    # thicker outline for large craters
    linewidth = 2.5 if (t == "crater" and ri > ABOVE_THRESHOLD) else 1.0
    circle = plt.Circle((xi, yi), ri, fill=False, edgecolor=edge_color, linewidth = linewidth, alpha=0.9)

    ax.add_patch(circle)

def distribution_vizualization(map_distance_meter, map_resolution, data, ABOVE_THREASHOLD):
    
    centers = data["center"] * map_distance_meter
    radii = data["radius"]
    types = data["type"]

    # Precompute
    is_crater = (types == "MHP_crater")
    is_large = radii > ABOVE_THREASHOLD

    # Build patches
    patches = [Circle((x, y), r) for (x, y), r in zip(centers, radii)]

    # Colors & alpha (vectorized)
    edge_colors = ["blue" if c else "red" for c in is_crater]
    alphas = [0.8 if (c and l) else 0.1 for c, l in zip(is_crater, is_large)]

    fig, ax = plt.subplots(figsize=(10,10))

    collection = PatchCollection(
        patches,
        facecolor=edge_colors,
        edgecolor=edge_colors,
        alpha=None  # handled per patch below
    )
    # Apply per-patch alpha
    collection.set_alpha(alphas)
    ax.add_collection(collection)

    ax.set_xlim(0,map_distance_meter)
    ax.set_ylim(0,map_distance_meter)
    ax.legend(f"Resolution {map_resolution} m per Pixel")
    ax.set_xlabel("Meter")
    ax.set_ylabel("Meter")
    ax.set_aspect('equal')
    ax.set_title("Map of Lunar Surface")
    plt.figtext(0.1,0.9,f"Map Size is {map_distance_meter} meter")
    legend_large_crater = plt.Circle((0,0), 1, edgecolor='blue', fill=True, alpha=0.6, label='Crater (above resolution)')
    legend_small_crater = plt.Circle((0,0), 1, edgecolor='blue', fill=True, alpha=0.2, label='Crater (below resolution)')
    legend_other = plt.Circle((0,0), 1, edgecolor='red', fill=True, alpha=0.2, label='Boulders')
    ax.legend(handles=[legend_small_crater, legend_large_crater, legend_other], loc="upper right")

    return ax

def distribution_vizualization_layers(map_distance_meter, map_resolution, data,
                               show_MHP=True, show_craters=True, show_boulders=True,
                               pit_outline=None, show_pit=True,
                               faint_radius_threshold_m=2):

    centers = data["center"] * map_distance_meter
    radii = data["radius"]
    types = data["type"]

    fig, ax = plt.subplots(figsize=(10,10))

    def add_layer(mask, color_visible, color_small, alpha_visible=0.6, alpha_small=0.5):
        if not np.any(mask):
            return

        c = centers[mask]
        r = radii[mask]

        # visibility based on resolution
        visible = r >= faint_radius_threshold_m

        patches = [Circle((x, y), ri) for (x, y), ri in zip(c, r)]

        colors = [color_visible if v else color_small for v in visible]
        alphas = [alpha_visible if v else alpha_small for v in visible]

        collection = PatchCollection(
            patches,
            facecolor=colors,
            edgecolor=colors
        )
        collection.set_alpha(alphas)
        ax.add_collection(collection)

    # --- Layers ---
    if show_MHP:
        add_layer(types == "MHP_crater", color_visible="purple", color_small="violet")

    if show_craters:
        add_layer(types == "crater", color_visible="blue", color_small="lightblue")

    if show_boulders:
        add_layer(types == "boulder", color_visible="red", color_small="salmon")

    if show_pit and pit_outline is not None:
        pit_m = pit_outline * map_distance_meter
        ax.add_patch(MplPolygon(pit_m, closed=True, facecolor="black", edgecolor="black", alpha=0.8))

    ax.set_aspect('equal')
    ax.set_xlim(0,map_distance_meter)
    ax.set_ylim(0,map_distance_meter)
    ax.set_xlabel("Meter")
    ax.set_ylabel("Meter")
    return ax

def mask_generator(map_distance_meter,map_resolution,centers_normalized,radii_meter,blowup):

    # Scale the crator_center location to the map size
    centers = centers_normalized * map_distance_meter * map_resolution
    radii_meter = radii_meter * map_resolution

    # total amount of pixels
    map_pixels = map_distance_meter * map_resolution
    
    image = np.zeros((map_pixels, map_pixels))

    # returns tensor with two matrices whre the first row start at 0 and is filled up to the bottom
    # spacing between is 1, but this should be modulated
    yy, xx = np.mgrid[:map_pixels, :map_pixels]

    # set blow up factor to 1.1
    for (cx, cy), r in zip(centers, radii_meter):
        # check that circle intersects with pixel area
        mask = (xx - cx)**2 + (yy - cy)**2 <= (r*blowup)**2
        image[mask] = 1  # =+1 accumulate overlapping circles

    return image


def map_vizalization(image, map_resolution, map_distance_meter):
    # Plot in pixel coordinates
    fig, ax = plt.subplots(figsize=(10,10))
    plt.imshow(image, cmap='binary', origin='lower')

    ax.set_xlim(0,map_distance_meter * map_resolution)
    ax.set_ylim(0,map_distance_meter * map_resolution)
    ax.set_xlabel("Pixels")
    ax.set_ylabel("Pixels")
    ax.set_aspect('equal')
    ax.set_title("Rasterized map of Lunar Surface")
    plt.figtext(0.1,0.9,f"Map Size is {map_distance_meter} meter")
    return ax

def distribution_vizualization2(map_distance_meter,map_resolution,centers_normalized, radii_meter):
    # Scale the crator_center location to the map size
    centers = centers_normalized * map_distance_meter *map_resolution
    radii_meter = radii_meter * map_resolution
    
    # for plotting circles
    x_px = centers[:,0] 
    y_px = centers[:,1] 

    # Plot in pixel coordinates
    fig, ax = plt.subplots(figsize=(10,10))
    #plt.imshow(image, cmap='binary', origin='lower')

    for xi, yi, ri in zip(x_px,y_px,radii_meter):
        circle = plt.Circle((xi,yi), ri, fill=True, edgecolor='blue',alpha = 0.2)
        ax.add_patch(circle)

    ax.set_xlim(0,map_distance_meter * map_resolution)
    ax.set_ylim(0,map_distance_meter * map_resolution)
    ax.set_aspect('equal')
    ax.set_title("Gridmap")
    return


def circle_intersects_cell(cx, cy, r, cell_x, cell_y, cell_size):
    # closest point in the cell to the circle center
    closest_x = max(cell_x, min(cx, cell_x + cell_size))
    closest_y = max(cell_y, min(cy, cell_y + cell_size))

    dx = cx - closest_x
    dy = cy - closest_y

    return dx*dx + dy*dy <= r*r


