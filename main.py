import math
import random

import numpy as np
import sumolib
import traci
import traci.constants as tc

from cost_calculation import calc_costs
from models.vehicle import Vehicle
from synthesise_utils import calc_dist, clockwise_2d_angle
from trajectory_calculation import find_first_lane_in_direction, calc_intersections, calc_trajectory_lane, \
    calc_trajectory_lane_2, calc_trajectory_street
from settings import *

class Simulation:
    def __init__(self, junction_id, network_file, beta_loc, beta_v, beta_veh_gap, beta_h):
        self.junction_id = junction_id
        self.net = sumolib.net.readNet(network_file, withInternal=True, withPrograms=True)  # parse network file
        self.intersections = calc_intersections(self.net, self.junction_id)  # calc sub-intersectios a priori (cf. vehicle.py)
        self.vehicles = []
        self.emergency_state_vehicles = []  # cf. vehicle.py
        self.dt = 1  # resolution of simulation. Default: 1 second.
        self.sumo_cmd = [sumoBinary, '-c', directory + '/' + cnfg_file, '--fcd-output',
                         directory + '/grid_search_results/fcd_out_%.2f_%.2f_%.2f_%d.xml' % (
                             beta_loc, beta_v, beta_veh_gap, beta_h),
                         "--device.fcd.explicit", "vehDist", "--fcd-output.geo"]
        self.weights = (beta_loc, beta_v, beta_veh_gap, beta_h)  # hyperparameter betas. Cf. thesis eq 4.2

    def is_id_in_vehicles(self, v_id):
        for v in self.vehicles:
            if v_id == v.vehicle_id:
                return True
        return False

    def get_vehicle(self, v_id):
        for v in self.vehicles:
            if v_id == v.vehicle_id:
                return v
        return None

    def remove_vehicle(self, v_id):
        for i in range(len(self.vehicles)):
            if self.vehicles[i].vehicle_id == v_id:
                self.vehicles.pop(i)
                return True
        return False

    def handle_emergency_state_vehicle(self, vehicle, step):
        if vehicle.emergency_stop is None:
            vehicle.emergency_stop = step
        else:
            if vehicle.emergency_stop + 2 >= step:  # if ES happens twice in 3 sim steps, cyclist is stuck and needs to be removed from the simulation
                traci.vehicle.remove(vehicle.vehicle_id)
                self.remove_vehicle(vehicle.vehicle_id)
            else:
                vehicle.emergency_stop = step

    def get_tl_state(self, tl, direction, current_edge):
        tl_phases = self.net.getTLS(tl.getID()).getPrograms()['0'].getPhases()
        tl_link_index = find_first_lane_in_direction(direction, current_edge.getLanes()[1:])[1].getTLLinkIndex()
        current_tl_phase_index = traci.trafficlight.getPhase(tl.getID())
        return tl_phases[current_tl_phase_index].state[tl_link_index].lower()

    def find_corresponding_sub_intersection(self, edge):  # finds relevant sub-intersection
        for sub_intersection in self.intersections:
            if sub_intersection[0] is edge:
                return sub_intersection
        return None

    def is_turning_left(self, vehicle_id, current_edge):
        p1 = current_edge.getShape(False)[0]
        p2 = current_edge.getShape(False)[-1]
        current_edge_dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])

        route = traci.vehicle.getRoute(vehicle_id)
        next_edge_id = route[[index + 1 for index, edge in enumerate(route) if edge == current_edge.getID()][0]]
        next_edge = self.net.getEdge(next_edge_id)

        p1 = next_edge.getShape(False)[0]
        p2 = next_edge.getShape(False)[-1]
        next_edge_dir_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])

        angle = clockwise_2d_angle(next_edge_dir_vector, current_edge_dir_vector)
        return -120 < angle < -60



    def gen_action_paris(self, vehicle):  # generate possible actions. Cf. thesis 5.3.2 "action space"
        a_step = 0.5
        a_max = (math.ceil(traci.vehicle.getAccel(vehicle.vehicle_id) / a_step) * a_step) + 0.1
        d_max = -traci.vehicle.getDecel(vehicle.vehicle_id)
        accels = np.arange(d_max, a_max, a_step)
        angles = np.arange(-45, 46, 15)
        mesh = np.array(np.meshgrid(accels, angles))
        combinations = mesh.T.reshape(-1, 2)
        return combinations

    def perform(self, v, action):  # vars with _ are the updated vars after the realization of the action
        a = action[0]
        delta_angle = action[1]

        # ensure that velo is always >= 0
        if v.velo + 0.5 * a * self.dt ** 2 < 0:
            a = - 2 * (v.velo / self.dt ** 2)

        angle_ = (v.angle + delta_angle) % 360

        x_ = v.x + v.velo * math.sin(math.radians(angle_)) * self.dt + 0.5 * math.sin(
            math.radians(angle_)) * a * self.dt ** 2
        y_ = v.y + v.velo * math.cos(math.radians(angle_)) * self.dt + 0.5 * math.cos(
            math.radians(angle_)) * a * self.dt ** 2

        if v.velo + a * self.dt == 0:  # prevents cyclists from spinning on the spot
            angle_ = v.angle

        traci.vehicle.setSpeed(v.vehicle_id, v.velo + a * self.dt)
        traci.vehicle.moveToXY(v.vehicle_id, v.trajectory[v.traj_step][4], v.trajectory[v.traj_step][5], x_, y_,
                               angle=angle_, keepRoute=v.keep_route)

    def run(self):

        traci.start(self.sumo_cmd)  # connect to SUMO
        traci.junction.subscribeContext(self.junction_id, tc.CMD_GET_VEHICLE_VARIABLE, 200,
                                        [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_ANGLE])  # Everything 200m around the intersection is observed
        random.seed(3)
        step = 0
        while step < 8000:  # replace with traci.simulation.getMinExpectedNumber() > 0 for general use

            traci.simulationStep()  # perform simulation step

            status = traci.junction.getContextSubscriptionResults(self.junction_id)  # retrieve status of intersection
            emergency_state_vehicles = list(traci.simulation.getEmergencyStoppingVehiclesIDList())  # retrieve status of ES vehicles (to prevent get stuck bug (cf. vehicle.py))

            for v_id, value in status.items():
                if 'car' in v_id:  # cars are not steered by model
                    continue

                if not self.is_id_in_vehicles(v_id):  # initialize new cyclists
                    maneuver = random.choices([('lane', 0, 's'), ('street', 4, 'l')], pathfinding_distribution, k=1)[0]
                    v = Vehicle(v_id,
                                value[tc.VAR_POSITION][0], value[tc.VAR_POSITION][1],
                                value[tc.VAR_SPEED],
                                value[tc.VAR_ANGLE],
                                None if traci.vehicle.getLeader(v_id) is None else traci.vehicle.getLeader(v_id)[1],
                                maneuver,
                                traci.vehicle.getRouteID(v_id)
                                )
                    self.vehicles.append(v)
                else:  # update known cyclists
                    v = self.get_vehicle(v_id)
                    v.x = value[tc.VAR_POSITION][0]
                    v.y = value[tc.VAR_POSITION][1]
                    v.velo = value[tc.VAR_SPEED]
                    v.angle = value[tc.VAR_ANGLE]
                    v.dir_x = math.cos(math.radians(v.angle))
                    v.dir_y = math.sin(math.radians(v.angle))
                    lat_gap_vehicle = traci.vehicle.getLeader(v_id)
                    v.lat_gap_vehicle = None if lat_gap_vehicle is None else lat_gap_vehicle[1]

                if v.vehicle_id in emergency_state_vehicles:  # handle possible ES cyclists (edge case. Occurs rarely)
                    self.handle_emergency_state_vehicle(v, step)

                current_edge = self.net.getEdge(traci.vehicle.getRoadID(v.vehicle_id))  # retrieve cyclist's current_edge

                if (v.active is None) and (v.trajectory is None) and (current_edge.getToNode().getID() == self.junction_id):  # True if cyclist's first step relevant to the model
                    if self.is_turning_left(v.vehicle_id, current_edge):
                        v.active = True
                        v.sub_intersection = self.find_corresponding_sub_intersection(current_edge)  # find sub-intersection (cf. vehicle.py)
                        v.destination_edge = find_first_lane_in_direction('l', current_edge.getLanes())[
                            1].getTo()  # as this model enables left turns, the destination edge is always in direction 'l'
                        if v.maneuver == 'lane':  # calc guideline $G$ (cf. thesis)
                            v.trajectory = calc_trajectory_lane(self.net, current_edge, v, self.dt)
                        else:
                            v.trajectory = calc_trajectory_street(self.net, current_edge, v, self.dt)
                    else:  # only left turning cyclists are steered by the model
                        v.active = False

                if v.active and current_edge == v.destination_edge and calc_dist(*current_edge.getShape()[-1],
                                                                                 *traci.vehicle.getPosition(
                                                                                     v.vehicle_id)) < 2 * traci.vehicle.getMaxSpeed(
                    v.vehicle_id) * self.dt:  # return cyclist to SUMO if intersection was passed
                    traci.vehicle.setRouteID(v.vehicle_id, v.route_id)
                    v.active = False

                if v.active:
                    next_tl = current_edge.getTLS()
                    if v.tl is None:  # Hereby, only the relevant TL of the junction is considered. Possible other TLs are ignored.
                        v.tl = next_tl

                    if (v.tl is not None) and (current_edge.getTLS() == v.tl):

                        if v.stop_flag and self.get_tl_state(v.tl, v.tl_direction, current_edge) == 'g':
                            if v.maneuver == 'lane':
                                v.trajectory = calc_trajectory_lane(self.net, current_edge, v, self.dt)
                            else:
                                v.trajectory = calc_trajectory_street(self.net, current_edge, v, self.dt)
                            v.traj_step = 0  # reset step count in guideline
                            v.stop_flag = False  # reset stop flag

                    if (not v.once) and v.maneuver == 'lane' and current_edge.getFunction() == 'internal' and self.get_tl_state(
                        v.tl,
                        v.tl_direction,
                        v.sub_intersection[
                            1]) == 'g':  # if cyclist uses 'lane' path and second TL turns green, calc trajectory_lane_2
                        v.trajectory = calc_trajectory_lane_2(self.net, current_edge, v, self.dt)
                        v.stop_flag = False
                        v.traj_step = 0
                        v.once = True

                    costs = []

                    if v.tl is not None and current_edge.getTLS() == v.tl and self.get_tl_state(v.tl, v.tl_direction,
                                                                                                current_edge) == 'r':
                        lat_gap_tl = current_edge.getLength() - traci.vehicle.getLanePosition(v.vehicle_id)  # gap to TL
                    else:
                        lat_gap_tl = None  # gap to TL is not relevant

                    for possible_action in self.gen_action_paris(v):  # cf. thesis equation 4.1
                        costs.append(np.array(
                            [calc_costs(self.weights, v, *possible_action, lat_gap_tl, self.dt), possible_action]))
                    costs = np.array(costs)
                    min_cost = costs[np.argmin(costs[:, 0][:, 1])][0][1]

                    if min_cost == float('inf'):  # brake super hard if min_cost == infinite (edge case)
                        action = np.array([-10., 0.])
                    else:
                        action = costs[np.argmin(costs[:, 0][:, 1])][1]
                    traj_step = costs[np.argmin(costs[:, 0][:, 1])][0][0]  # obtain $t*$ (cf. thesis eq. 4.2)

                    v.traj_step = v.trajectory[int(traj_step)][7]  # obtain $g_t*$ (cf. thesis eq. 4.2)
                    self.perform(v, action)

            step += 1
        traci.close()

# Specify intervals for grid search for beta params
beta_loc = np.arange(1, 2.1, 0.5)
beta_v = np.arange(1, 2.1, 0.5)
beta_veh_gap = np.arange(1, 3.1, 1)
beta_h = np.arange(5, 21, 5)


if not eval_hyperparameters:
    sim = Simulation(junction_id, network_file, beta_loc=2, beta_v=1, beta_veh_gap=1, beta_h=15)
    sim.run()
else:
    # Grid search for best beta params. Note, evaluation is not done here. Must be conducted externally (im_eval/hyperparameters) with the result obtained here.
    for b_l in beta_loc:
        for b_v in beta_v:
            for b_g in beta_veh_gap:
                for b_s in beta_h:
                    sim = Simulation(junction_id, network_file, beta_loc=b_l, beta_v=b_v, beta_veh_gap=b_g, beta_h=b_s)
                    try:
                        sim.run()
                    except Exception as e:
                        traci.close(False)
                        print(e, '/n Error in b_p:%.2f and b_v:%.2f' % (b_l, b_v))
