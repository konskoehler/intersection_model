import math
import numpy as np
from synthesise_utils import calc_dist


# use 3-angle "law of angle" math to calc the gap between cyclist and obstacle
# A = adjacent, O = opposite, H = hypotenuse
#    /|\
#   / | \
# H/ O|O'\H'
# /___|___\
#   A   A'
def calc_updated_lat_gap(delta_angle, delta_distance, current_lat_gap):
    if current_lat_gap is None:
        return None
    elif current_lat_gap < 0:
        return 0
    # A = cos(alpha) * H
    A = math.cos(math.radians(delta_angle)) * delta_distance
    # O = sin(alpha) * H
    O = math.sin(math.radians(delta_angle)) * delta_distance
    # A' = prev_lat_gap - A
    A_ = current_lat_gap - A
    # O' = O
    # H' = sqrt(A'^2 + O'^2)
    try:
        res = math.sqrt(A_ ** 2 + O ** 2)
    except:
        res = 50  # arbitrary high number < inf
    return res


def get_lat_gap_veh_cost(lat_gap):
    if lat_gap is None:
        return 0
    elif lat_gap > 20:  # irrelevant to cyclists
        return 0
    elif lat_gap <= 0:
        return float('inf')
    else:
        return 1 / lat_gap


def get_lat_gap_tl_cost(lat_gap):
    if lat_gap is None:
        return 0
    elif lat_gap > 2:
        return 0
    else:
        return float('inf')


# cf. thesis eq. 4.2
def calc_cost_step(arr, weights, x_, y_, velo_, lat_gap_veh_, lat_gap_tl_, traj_step):
    res = (arr[7], calc_dist(x_, y_, arr[0], arr[1]) * weights[0] + abs(velo_ - arr[2]) * weights[1] + get_lat_gap_veh_cost(
        lat_gap_veh_) * weights[2] + get_lat_gap_tl_cost(lat_gap_tl_) + (1 if arr[7] <= traj_step else 0) * weights[3])
    return res


# cf. thesis eq. 4.1
def calc_costs(weights, vehicle, a, delta_angle, lat_gap_tl, dt):
    if vehicle.velo == 0 and delta_angle != 0:
        return 0, float('inf')

    velo_ = vehicle.velo + a * dt
    if velo_ < 0:
        velo_ = 0

    if vehicle.velo + 0.5 * a * dt ** 2 < 0:
        a = - 2 * (vehicle.velo / dt ** 2)

    angle_ = vehicle.angle + delta_angle
    dir_ = [math.sin(math.radians(angle_)), math.cos(math.radians(angle_))]

    x_ = vehicle.x + vehicle.velo * dt * dir_[0] + 0.5 * a * dt ** 2 * dir_[0]
    y_ = vehicle.y + vehicle.velo * dt * dir_[1] + 0.5 * a * dt ** 2 * dir_[1]

    lat_gap_veh_ = calc_updated_lat_gap(delta_angle, velo_ * dt, vehicle.lat_gap_vehicle)
    lat_gap_tl_ = calc_updated_lat_gap(delta_angle, velo_ * dt, lat_gap_tl)

    #  only calculate costs for last 3 and next 7 steps of guideline. Reduces computational load
    costs = np.apply_along_axis(calc_cost_step, 1, vehicle.trajectory[
                                                   vehicle.traj_step - 3 if vehicle.traj_step > 2 else 0:
                                                   vehicle.traj_step + 7 if vehicle.traj_step + 7 < len(
                                                       vehicle.trajectory) else len(vehicle.trajectory) - 1],
                                weights,
                                x_,
                                y_, velo_,
                                lat_gap_veh_, lat_gap_tl_, vehicle.traj_step)

    return costs[np.argmin(costs[:, 1])][0], np.min(costs[:, 1])
