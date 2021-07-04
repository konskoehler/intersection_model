import math
import numpy as np
import pandas as pd


def resize_vector_to_one(vec):
    length = np.linalg.norm(vec)
    if length == 0.0:
        return np.array([0., 0.])
    else:
        return vec / length


def calc_dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def insert_empty_rows(df):
    data = df.values
    for i in range(1):
        nans = np.where(np.empty_like(data), np.nan, np.nan)
        data = np.hstack([nans, data])
    return pd.DataFrame(data.reshape(-1, df.shape[1]), columns=df.columns)


def clockwise_2d_angle(v1, v2):
    unit_v1 = resize_vector_to_one(v1)
    unit_v2 = resize_vector_to_one(v2)
    return math.degrees(math.atan2(np.cross(unit_v1, unit_v2), np.dot(unit_v1, unit_v2)))


# calc intersection of 2 lines
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y
