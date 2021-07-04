import math


class Vehicle:
    def __init__(self, vehicle_id, x, y, velo, angle, lat_gap, maneuver, route_id):
        self.active = None  # only active vehicles are steered by the model
        self.traj_step = 0  # step inside the guideline (trajectory)
        self.vehicle_id = vehicle_id
        self.x = x
        self.y = y
        self.velo = velo
        self.angle = angle
        self.dir_x = math.cos(math.radians(self.angle))
        self.dir_y = math.sin(math.radians(self.angle))
        self.trajectory = None  # cf. guideline $G$ from the thesis
        self.lat_gap_vehicle = lat_gap  # lateral gap to next vehicle in meter
        self.tl = None  # traffic light which is relevant to vehicle
        self.stop_flag = None  # indicates whether the vehicle needs to brake
        self.maneuver = maneuver[0]  # either 'lane' or 'street' (cf. thesis)
        self.keep_route = maneuver[1]  # important for traci.vehicle.moveToXY (cf. traci docs)
        self.tl_direction = maneuver[2]  # relevant signal of the tl (s, r, or l)
        self.once = False  # helps to only calc trajectory_2 (in 'lane' path) once
        self.sub_intersection = None  # every intersection has 4 sub-intersections where the straight going bike lanes intersect. Bicycles wait here for the straight going traffic respectively the traffic light phase.
        self.destination_edge = None
        self.route_id = route_id
        self.emergency_stop = None  # sometimes cyclists get stuck as they perform so called emergency stops. If they perform 2 consecutive emergency stops, they get stuck. This var remembers the step of the last ES.
