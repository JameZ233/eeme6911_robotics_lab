import numpy as np

def estimate(points):
    # Parse corner points from input dictionary
    pts = np.array([[p["x"], p["y"]] for p in points], dtype=float)
    xs, ys = pts[:,0], pts[:,1]

    # Compute Height
    y_top, y_bottom = np.min(ys), np.max(ys)
    height = y_bottom - y_top

    # Split left and right points using median x
    x_med = np.median(xs)
    left  = pts[xs <= x_med]
    right = pts[xs >  x_med]

    # Sort by y and pair points by rank
    L = left[left[:,1].argsort()]
    R = right[right[:,1].argsort()]
    k = min(len(L), len(R))
    L, R = L[:k], R[:k]

    # Compute midpoints and take median for axis
    mid_x = 0.5 * (L[:,0] + R[:,0])
    x_axis = np.median(mid_x)



    return x_axis, height

# Sample input points got from topic subscription
points = [
    {"x":283.0,"y":276.0},
    {"x":355.0,"y":275.0},
    {"x":283.0,"y":85.0},
    {"x":356.0,"y":87.0},
    {"x":329.0,"y":278.0},
    {"x":309.0,"y":278.0},
    {"x":347.0,"y":81.0},
    {"x":295.0,"y":80.0},
    {"x":331.0,"y":78.0},
    {"x":308.0,"y":78.0},
    {"x":356.0,"y":265.0},
    {"x":356.0,"y":239.0},
    {"x":282.0,"y":239.0},
]
# Call the function
height, x_axis = estimate(points)
print("height =", height)
print("vertical_axis_position =", x_axis)
