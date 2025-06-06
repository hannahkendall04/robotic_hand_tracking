import numpy as np

def calibrate_index(circle_center, radius, index_tip, screen_width, screen_height):

    x_index = int(index_tip.x * screen_width)
    y_index = int(index_tip.y * screen_height)
    index_point = np.array([x_index, y_index])
    circle_point = np.array([circle_center[0], circle_center[1]])

    dist = np.linalg.norm(circle_point - index_point)
    if dist < radius:
        return True 
    else:
        return False


def get_params(index_tip, thumb_tip):

    index_point = np.array([index_tip.x, index_tip.y, index_tip.z])
    thumb_point = np.array([thumb_tip.x, thumb_tip.y, thumb_tip.z])

    grip_dist = np.linalg.norm(index_point - thumb_point)

    return [index_point, thumb_point, grip_dist]