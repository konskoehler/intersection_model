import math
import numpy as np
import pandas as pd
import traci
from shapely.geometry import LineString
from synthesise_utils import clockwise_2d_angle, line_intersection, calc_dist, resize_vector_to_one, insert_empty_rows


### Methods from this script are used for the synthesis of guideline $G$ (cf. thesis). Here, $G$ is called "trajectory"
### For the debugging of these methods, it is recommended to run SUMO's netedit in parallel since netedit provides x and
### y coordinates when hovering over the edges. Thereby, the following code should be understandable.


def find_second_crossing_edge(current_edge, junc):
    for e in junc.getIncoming():
        current_edge_bike_lane, _ = find_first_lane_in_direction('s', current_edge.getLanes())
        p1 = current_edge_bike_lane.getShape()[0]
        p2 = current_edge_bike_lane.getShape()[-1]
        current_edge_dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])

        if not e.isSpecial() and e is not current_edge:
            lane, junc_conn = find_first_lane_in_direction('s', e.getLanes())
            p1 = lane.getShape()[0]
            p2 = lane.getShape()[-1]
            dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
            angle = clockwise_2d_angle(dir_vector, current_edge_dir_vector)
            if -120 < angle < -60:
                return e
    return None


def find_first_lane_in_direction(direction, lanes):
    for lane in lanes:
        for conn in lane.getOutgoing():
            if conn.getDirection() == direction:
                return lane, conn
    return None, None


def get_segment_of_dist(segments, dist):
    cum_dist = 0
    for i, seg in enumerate(segments):
        if cum_dist <= dist <= (cum_dist + seg[2]):
            return seg
        cum_dist += seg[2]
    return segments[-1]  # if dist exceeds cumulative length of segments, return last


def calc_intersections(net, junction_id):
    junc = net.getNode(junction_id)

    orthogonal_pairs = []
    for current_edge in junc.getIncoming():
        if not current_edge.isSpecial():
            current_edge_bike_lane, _ = find_first_lane_in_direction('s', current_edge.getLanes())
            if current_edge_bike_lane is None:
                continue
            p1 = current_edge_bike_lane.getShape()[0]
            p2 = current_edge_bike_lane.getShape()[-1]
            current_edge_dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])

            for e in junc.getIncoming():
                if not e.isSpecial() and e is not current_edge:
                    lane, junc_conn = find_first_lane_in_direction('s', e.getLanes())
                    if lane is None:
                        continue
                    p1 = lane.getShape()[0]
                    p2 = lane.getShape()[-1]
                    dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
                    angle = clockwise_2d_angle(dir_vector, current_edge_dir_vector)
                    if -120 < angle < -60:
                        orthogonal_pairs.append((current_edge, e))
                        break

    res = []
    for pair in orthogonal_pairs:
        edge = pair[0]
        lane, junc_conn = find_first_lane_in_direction('s', edge.getLanes())

        junc_internal_edge = net.getEdge(':%s_%i' % (edge.getToNode().getID(), junc_conn.getTLLinkIndex()))

        x_edge = find_second_crossing_edge(edge, junc)
        x_lane, x_junc_conn = find_first_lane_in_direction('s', x_edge.getLanes())
        x_junc_internal_edge = net.getEdge(':%s_%i' % (x_edge.getToNode().getID(), x_junc_conn.getTLLinkIndex()))

        coords = junc_internal_edge.getShape()
        segments = []
        for i in range(len(coords) - 1):
            segments.append((coords[i], coords[i + 1]))

        x_coords = x_junc_internal_edge.getShape()
        x_segments = []
        for i in range(len(x_coords) - 1):
            x_segments.append((x_coords[i], x_coords[i + 1]))

        for seg in segments:
            for x_seg in x_segments:
                if LineString(seg).intersects(LineString(x_seg)):
                    line1 = [seg[0], seg[-1]]
                    line2 = [x_seg[0], x_seg[-1]]
                    res.append((edge, pair[1], line_intersection(line1, line2)))
    return res


def calc_expected_phase_duration(phase, tl_id, tl_phases, lane_index, current_phase_index):
    exp_dur = 0
    for i in range(current_phase_index + 1, len(tl_phases)):
        if not tl_phases[i].state[lane_index].lower() == phase:
            break
        else:
            exp_dur += tl_phases[i].duration

    if tl_phases[current_phase_index].state[lane_index].lower() == phase:
        exp_dur += (traci.trafficlight.getNextSwitch(tl_id) - traci.simulation.getTime())

    return exp_dur


def get_dist_until_edge(segments, edge):
    cum_dist = 0
    for seg in segments:
        if seg[5] is edge:
            break
        cum_dist += seg[2]
    return cum_dist


def postprocess_trajectory(traj, segments):
    df = pd.DataFrame(traj, columns=['x', 'y', 'velo', 'dist', 'edge', 'lane'])

    df = insert_empty_rows(df)  # enhance the resolution of the calculated trajectory
    df[['x', 'y', 'dist']] = df[['x', 'y', 'dist']].astype(float).interpolate()

    df = df[~df['x'].isnull()]
    df['velo'] = df['velo'].fillna(method='ffill')
    df['edge'] = df.apply(lambda x: get_segment_of_dist(segments, x['dist'])[5], axis=1)  # find edge_id
    df['lane'] = df.apply(lambda x: get_segment_of_dist(segments, x['dist'])[6], axis=1)  # find lane_id

    df['x_delta'] = df['x'] - df.shift(1)['x']
    df['y_delta'] = df['y'] - df.shift(1)['y']
    df['dir_angle'] = df.apply(lambda x: ((math.atan2(x.y_delta, x.x_delta) * 180 / math.pi) * -1 + 90) % 360, axis=1)

    df['enumeration'] = range(len(df))
    return df.drop(['x_delta', 'y_delta'], axis=1).to_numpy()


def calc_trajectory_street(net, current_edge, vehicle, dt):
    v = vehicle.velo

    segments = []
    start = traci.vehicle.getPosition(vehicle.vehicle_id)
    lane, junc_conn = find_first_lane_in_direction('l', current_edge.getLanes()[1:])  ## bicycle lane is left out
    end = lane.getShape()[-1]
    segments.append((start, end, calc_dist(start[0], start[1], end[0], end[1]),
                     resize_vector_to_one([end[0] - start[0], end[1] - start[1]]), 0, current_edge.getID(),
                     lane.getIndex()))

    tl_phases = net.getTLS(junc_conn.getTLSID()).getPrograms()['0'].getPhases()
    current_tl_phase_index = traci.trafficlight.getPhase(junc_conn.getTLSID())
    time_until_red = calc_expected_phase_duration('g', junc_conn.getTLSID(), tl_phases, junc_conn.getTLLinkIndex(),
                                                  current_tl_phase_index)
    distance_until_tl = traci.vehicle.getNextTLS(vehicle.vehicle_id)[0][2]

    if vehicle.stop_flag or v * time_until_red > distance_until_tl:
        a = traci.vehicle.getAccel(vehicle.vehicle_id)
        vehicle.stop_flag = False
    else:
        a = -0.5 * (v ** 2) / distance_until_tl
        vehicle.stop_flag = True

    if not vehicle.stop_flag:
        junc_internal_edge = net.getEdge(':%s_%i' % (current_edge.getToNode().getID(), junc_conn.getTLLinkIndex()))
        points_on_junc = junc_internal_edge.getShape(includeJunctions=False)
        for i in range(len(points_on_junc) - 1):
            x1, y1 = points_on_junc[i]
            x2, y2 = points_on_junc[i + 1]
            segments.append((points_on_junc[i], points_on_junc[i + 1], calc_dist(x1, y1, x2, y2),
                             resize_vector_to_one([x2 - x1, y2 - y1]), 1, junc_internal_edge.getID(), -1))

        last_edge = list(junc_internal_edge.getOutgoing().keys())[0]
        last_edge_start = last_edge.getLanes()[1].getShape()[0]
        last_edge_end = last_edge.getLanes()[0].getShape()[-1]

        ## the junc_internal_edge.getShape(includeJunctions=False) only contains half of the interal edge's coords. God knows why.
        ## That's why this heuristic was chosen, to link the last point of the internal edge with the first point of the
        ## next edge.
        x1, y1 = points_on_junc[i + 1]
        x2, y2 = last_edge_start
        segments.append((points_on_junc[i + 1], last_edge_start,
                         calc_dist(x1, y1, x2, y2),
                         resize_vector_to_one([x2 - x1, y2 - y1]), 1, junc_internal_edge.getID(), -1))

        segments.append((last_edge_start, last_edge_end,
                         calc_dist(last_edge_start[0], last_edge_start[1], last_edge_end[0],
                                   last_edge_end[1]),
                         resize_vector_to_one(
                             [last_edge_end[0] - last_edge_start[0], last_edge_end[1] - last_edge_start[1]]), 2,
                         last_edge.getID(), 0))

    np_segments = np.array(segments, dtype='object')
    traj = []

    dist = 0
    total_dist = sum(np_segments[:, 2])

    prev_seg = None
    while True:
        seg = get_segment_of_dist(np_segments, dist + v * dt + 0.5 * a * dt ** 2)
        if seg is None:
            break

        ## how to compare 2 np arrays with nested elements? np.equals and np.array_equals do not work
        if prev_seg is None or (prev_seg[0][0] != seg[0][0] and prev_seg[0][1] != seg[0][1]):
            x_ = seg[0][0]
            y_ = seg[0][1]
            prev_seg = seg

        dist = dist + v * dt + 0.5 * a * dt ** 2

        dir_ = seg[3]

        if v + a * dt <= 0:
            a = - 2 * (v / dt ** 2)

        x_ = x_ + dir_[0] * v * dt + 0.5 * a * dt ** 2 * dir_[0]
        y_ = y_ + dir_[1] * v * dt + 0.5 * a * dt ** 2 * dir_[1]

        if 0 < v + a * dt <= traci.vehicle.getMaxSpeed(vehicle.vehicle_id):
            v = v + a * dt

        traj.append((x_, y_, v, dist, seg[5], seg[6]))

        if dist > total_dist or (vehicle.stop_flag and v + a * dt <= 0):
            traj.append((x_, y_, 0, dist, seg[5], seg[6]))
            break

    return postprocess_trajectory(traj, np_segments)


def calc_trajectory_lane(net, current_edge, vehicle, dt):
    v = vehicle.velo

    segments = []
    lane, junc_conn = find_first_lane_in_direction('s', current_edge.getLanes())

    tl_phases = net.getTLS(junc_conn.getTLSID()).getPrograms()['0'].getPhases()

    current_tl_phase_index = traci.trafficlight.getPhase(junc_conn.getTLSID())

    start = traci.vehicle.getPosition(vehicle.vehicle_id)

    end = lane.getShape()[-1]

    segments.append((start, end, calc_dist(start[0], start[1], end[0], end[1]),
                     resize_vector_to_one([end[0] - start[0], end[1] - start[1]]), 0, current_edge.getID(),
                     lane.getIndex()))

    time_until_red = calc_expected_phase_duration('g', junc_conn.getTLSID(), tl_phases, junc_conn.getTLLinkIndex(),
                                                  current_tl_phase_index)

    distance_until_tl = calc_dist(start[0], start[1], end[0], end[1])

    if vehicle.stop_flag or v * time_until_red > distance_until_tl:
        a = traci.vehicle.getAccel(vehicle.vehicle_id)
        vehicle.stop_flag = False
    else:
        a = -0.5 * (v ** 2) / distance_until_tl
        vehicle.stop_flag = True

    if not vehicle.stop_flag:

        junc_internal_edge = net.getEdge(':%s_%i' % (current_edge.getToNode().getID(), junc_conn.getTLLinkIndex()))

        intersection_coords = vehicle.sub_intersection[2]
        points_on_junc = junc_internal_edge.getShape(includeJunctions=False).copy()

        points_on_junc.append(intersection_coords)
        arr = np.array(points_on_junc)
        x_spread = np.max(arr[:, 0]) - np.min(arr[:, 0])
        y_spread = np.max(arr[:, 1]) - np.min(arr[:, 1])
        if x_spread > y_spread:
            arr = sorted(arr, key=lambda x: x[0], reverse=(arr[0][0] > arr[-1][0]))
        else:
            arr = sorted(arr, key=lambda x: x[1], reverse=(arr[0][1] > arr[-1][1]))

        for i in range(np.where(arr == np.array(intersection_coords))[0][0]):
            x1, y1 = arr[i]
            x2, y2 = arr[i + 1]
            segments.append((arr[i], arr[i + 1], calc_dist(x1, y1, x2, y2),
                             resize_vector_to_one([x2 - x1, y2 - y1]), 1, junc_internal_edge.getID(), -1))

    np_segments = np.array(segments, dtype='object')
    section_0_dist = sum(np_segments[(np_segments[:, 4] == 0)][:, 2])
    section_1_dist = sum(
        np_segments[(np_segments[:, 4] == 1)][:, 2])  ## sum distance of all segments in section 1.
    start_break_dist = section_0_dist + section_1_dist * 0.5  # start breaking in the middle of section 1.

    traj = []

    dist = 0
    total_dist = sum(np_segments[:, 2])
    breaking = False

    x_ = np_segments[0][0][0]
    y_ = np_segments[0][0][1]
    while True:
        seg = get_segment_of_dist(np_segments, dist + v * dt + 0.5 * a * dt ** 2)
        if not vehicle.stop_flag and seg[
            4] == 1 and not breaking and dist + v * dt + 0.5 * a * dt ** 2 >= start_break_dist:
            distance_until_second_tl = sum(np_segments[:, 2]) - dist
            a = -0.5 * (v ** 2) / distance_until_second_tl
            breaking = True

        dist = dist + v * dt + 0.5 * a * dt ** 2

        dir_ = seg[3]

        if v + a * dt <= 0:
            a = - 2 * (v / dt ** 2)

        x_ = x_ + dir_[0] * v * dt + 0.5 * a * dt ** 2 * dir_[0]
        y_ = y_ + dir_[1] * v * dt + 0.5 * a * dt ** 2 * dir_[1]

        if 0 < v + a * dt <= traci.vehicle.getMaxSpeed(vehicle.vehicle_id):
            v = v + a * dt

        traj.append((x_, y_, v, dist, seg[5], seg[6]))

        if dist > total_dist or ((breaking or vehicle.stop_flag) and v + a * dt <= 0):
            traj.append((x_, y_, 0, dist, seg[5], seg[6]))
            break

    return postprocess_trajectory(traj, np_segments)


def calc_trajectory_lane_2(net, current_edge, vehicle, dt):
    v = vehicle.velo
    a = traci.vehicle.getAccel(vehicle.vehicle_id)

    segments = []

    intersection_coords = vehicle.sub_intersection[2]

    start = traci.vehicle.getPosition(vehicle.vehicle_id)
    segments.append((start, intersection_coords,
                     calc_dist(start[0], start[1], intersection_coords[0], intersection_coords[1]),
                     resize_vector_to_one([intersection_coords[0] - start[0], intersection_coords[1] - start[1]]),
                     2, current_edge.getID(), -1))

    x_edge = vehicle.sub_intersection[1]
    x_lane, x_junc_conn = find_first_lane_in_direction('s', x_edge.getLanes())
    x_junc_internal_edge = net.getEdge(':%s_%i' % (x_edge.getToNode().getID(), x_junc_conn.getTLLinkIndex()))

    x_points_on_junc = x_junc_internal_edge.getShape(includeJunctions=False).copy()

    x_points_on_junc.append(intersection_coords)
    x_arr = np.array(x_points_on_junc)
    x_spread = np.max(x_arr[:, 0]) - np.min(x_arr[:, 0])
    y_spread = np.max(x_arr[:, 1]) - np.min(x_arr[:, 1])
    if x_spread > y_spread:
        x_arr = sorted(x_arr, key=lambda x: x[0], reverse=(x_arr[0][0] > x_arr[-1][0]))
    else:
        x_arr = sorted(x_arr, key=lambda x: x[1], reverse=(x_arr[0][1] > x_arr[-1][1]))

    for i in range(np.where(x_arr == np.array(intersection_coords))[0][0], len(x_arr) - 1):
        x1, y1 = x_arr[i]
        x2, y2 = x_arr[i + 1]
        segments.append((x_arr[i], x_arr[i + 1], calc_dist(x1, y1, x2, y2),
                         resize_vector_to_one([x2 - x1, y2 - y1]), 2, x_junc_internal_edge.getID(),
                         x_lane.getID()[-1]))

    last_edge = list(x_junc_internal_edge.getOutgoing().keys())[0]
    last_edge_start = last_edge.getLanes()[0].getShape()[0]
    last_edge_end = last_edge.getLanes()[0].getShape()[-1]
    segments.append((last_edge_start, last_edge_end,
                     calc_dist(last_edge_start[0], last_edge_start[1], last_edge_end[0], last_edge_end[1]),
                     resize_vector_to_one(
                         [last_edge_end[0] - last_edge_start[0], last_edge_end[1] - last_edge_start[1]]), 3,
                     last_edge.getID(), 0))

    np_segments = np.array(segments, dtype='object')
    traj = []

    dist = 0
    total_dist = sum(np_segments[:, 2])
    prev_seg = None

    while True:
        seg = get_segment_of_dist(np_segments, dist + v * dt + 0.5 * a * dt ** 2)
        if seg is None:
            break

        ## how to compare 2 np arrays with nested elements?
        if prev_seg is None or (prev_seg[0][0] != seg[0][0] and prev_seg[0][1] != seg[0][1]):
            x_ = seg[0][0]
            y_ = seg[0][1]
            prev_seg = seg

        dist = dist + v * dt + 0.5 * a * dt ** 2

        dir_ = seg[3]

        if v + a * dt <= 0:
            a = - 2 * (v / dt ** 2)

        x_ = x_ + dir_[0] * v * dt + 0.5 * a * dt ** 2 * dir_[0]
        y_ = y_ + dir_[1] * v * dt + 0.5 * a * dt ** 2 * dir_[1]

        if 0 < v + a * dt <= traci.vehicle.getMaxSpeed(vehicle.vehicle_id):
            v = v + a * dt

        traj.append((x_, y_, v, dist, seg[5], seg[6]))

        if dist > total_dist:
            break

    return postprocess_trajectory(traj, np_segments)
