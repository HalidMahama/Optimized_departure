import os
os.environ['SUMO_HOME']='/Users/mac/src/sumo-plexe-sumo-0.32.0'
import sys
import ccparams as cc
import random
import time
import planers
import sumolib
import traci
import math
import csv
import numpy as np
from shapely.geometry import Point, LineString, MultiPoint
from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running, validate_params, retrieve_vehicles, \
    filter_cacc_vehicles, get_dist_to_POI
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
# Length of vehicles
LENGTH = 4
# Inter-vehicluar gap
DISTANCE = 5.5
# cruise speed
SPEED = 45
# inimum platoon speed
PSPEED = 50
# Remove Vehicles
REMOVE_PARKING = 0x01

# PLANE STATES
# If the plane state is 0: there are no active planes to control
# If the plane state is 1: platoons are formed and lane changes are forbiden
# If the plane state is 2: vehicles are acc controlled and merging for the first time 
# If the plane state is 2: vehicles are acc controlled and merging for the first time
# State of platoons at departure
INSERT = 0
# Platooning state
PLATOONING = 1
# State for the departure of new platoons
INSERT2 =2
# N_VEHICLES: Number of vehicles that form primary platoon
N_VEHICLES = 24
# N_VEHICLES_GEN: Number of vehicles that form primary and secondary platoons
N_VEHICLES_GEN = 24
# pois: Points of Interests that represent Lane-Change Stations
pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
# PLAT_EDGES: Edges that support platooning
PLAT_EDGES = ["p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15",
              "n16", "p17", "n18", "p19"]
# ARR_EDGES: Last two edges of each route
ARR_EDGES = ["e0", "exit0", "e1", "exit1", "e2", "exit2", "e3", "exit3"]
# List of acceptable and complete routes 
main_routes = {"route_0_0":["source0", "s0", "n1", "p2", "n3", "e0", "exit0"], "route_0_1": ["source0", "s0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "e1", "exit1"], \
    "route_0_2" : ["source0", "s0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "e2", "exit2"],\
     "route_1_0" :["source1", "s1", "n6", "p7", "n8", "e1", "exit1"], "route_1_1":["source1", "s1", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "e2", "exit2"], "route_1_2" :["source1", "s1", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "e3", "exit3"], \
     "route_2_0" :["source2", "s2", "n11", "p12", "n13", "e2", "exit2"], "route_2_1": ["source2", "s2", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "e3", "exit3"], "route_2_2":["source2", "s2", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "e0", "exit0"],\
      "route_3_0":["source3", "s3", "n16", "p17", "n18", "e3", "exit3"], "route_3_1" : ["source3", "s3", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "e0", "exit0"], "route_3_2" :["source3", "s3", "n16", "p17", "n18", "p19", "p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "e1", "exit1"]}
list_of_leaders = []
# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "D", "-c", "cfg/freeway.sumo.cfg"]

def add_vehicles(n, batch_num, list_of_leaders, platoon_len, fromEdge, real_engine):
    """
    This function adds n number of vehicles as a platoon
    to 3 lanes of the source edge
    It also assigns the route of each member of a lane platoon
    based on the lane it is found on
    Param n : Total len of platoon
    Param batch num: nth batch of planes inserted so far starting
    for the particular lane
    Param platoon_len: len of platoon in secondary formation
    Param sourceEdge: Which sourcr to add vehicles to
    """
    start_from = n * batch_num  # start naming vehs from this number
    end_at = start_from + platoon_len  # stop at this number
    index = fromEdge.split("e")[1]
    if index == "0":
        exitEdges = ['exit0', 'exit1', 'exit2']
    elif index == "1":
        exitEdges = ['exit1', 'exit2', 'exit3']
    elif index == "2":
        exitEdges = ['exit2', 'exit3', 'exit1']
    else:
        exitEdges = ['exit3', 'exit1', 'exit2']
    # Add vehicles to lane 1 of the source edge and assign them 1st exit ramp
    for i in range(start_from, end_at):
        lane = 1
        vid = "v.%d" % i
        toEdge = exitEdges[0]
        route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)
    start_from = start_from + platoon_len  # start naming vehs from this num
    end_at = start_from + platoon_len  # stop here
    # Add vehicles to lane 2 of the source edge and assign them 2nd exit ramp
    for i in range(start_from, end_at):
        lane = 2
        vid = "v.%d" % i
        toEdge = exitEdges[1]
        route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)
    start_from = start_from + platoon_len  # start naming from this number
    end_at = start_from + platoon_len  # stop naming from this
    # Add vehicles to lane 3 of the source edge and assign them 3rd exit ramp
    for i in range(start_from, end_at):
        lane = 3
        vid = "v.%d" % i
        toEdge = exitEdges[2]
        route = route = "route_" + index + "_" + str(lane - 1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

def add_vehiclez(n, batch_num, list_of_leaders, switches, platoon_len, fromEdge, real_engine):
    """
    This function adds n number of vehicles as a platoon
    to 3 lanes of the source edge
    It also assigns the route of each member of a lane platoon
    based on the lane it is found on
    Param n : Total len of platoon
    Param batch num: nth batch of planes inserted so far starting
    for the particular lane
    Param platoon_len: len of platoon in secondary formation
    Param sourceEdge: Which sourcr to add vehicles to
    """

    start_from = n * batch_num  # start naming vehs from this number
    end_at = start_from + platoon_len  # stop at this number
    index = fromEdge.split("e")[1]
    inserted = 0
    if index == "0":
        exitEdges = ['exit0', 'exit1', 'exit2']
    elif index == "1":
        exitEdges = ['exit1', 'exit2', 'exit3']
    elif index == "2":
        exitEdges = ['exit2', 'exit3', 'exit1']
    else:
        exitEdges = ['exit3', 'exit1', 'exit2']
    # Add vehicles to lane 1 of the source edge and assign them 1st exit ramp
    if switches[0] == 'GREEN':
        for i in range(start_from, end_at):
            lane = 1
            vid = "v.%d" % i
            toEdge = exitEdges[0]
            route = "route_" + index + "_" + str(lane - 1)
            add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                        100, lane, SPEED, DISTANCE, real_engine)
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            change_lane(vid, lane)
        start_from = start_from + platoon_len  # start naming vehs from this num
        end_at = start_from + platoon_len  # stop here
        inserted +=1
    # Add vehicles to lane 2 of the source edge and assign them 2nd exit ramp
    if switches[1] == 'GREEN':
        for i in range(start_from, end_at):
            lane = 2
            vid = "v.%d" % i
            toEdge = exitEdges[1]
            route = "route_" + index + "_" + str(lane - 1)
            add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                        100, lane, SPEED, DISTANCE, real_engine)
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            change_lane(vid, lane)
        start_from = start_from + platoon_len  # start naming from this number
        end_at = start_from + platoon_len  # stop naming from this
        inserted += 1
    # Add vehicles to lane 3 of the source edge and assign them 3rd exit ramp
    if switches[2] == 'GREEN':
        for i in range(start_from, end_at):
            lane = 3
            vid = "v.%d" % i
            toEdge = exitEdges[2]
            route = route = "route_" + index + "_" + str(lane - 1)
            add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                        100, lane, SPEED, DISTANCE, real_engine)
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            change_lane(vid, lane)
        inserted +=1
    return inserted

def lane_gen(lanes_per_edge=4):
    """
    Generates the lane IDs for all lanes for which platooning is allowed
    It takes in number of lanes per edge and returns a list of all lanes 
    for every lane
    """
    edges = ["source0", "s0", "source1", "s1", "source2", "s2" , "source3", "s3", "p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19" ]
    lanes = []
    for edge in edges:
        if traci.edge.getLastStepVehicleIDs(edge) ==[]:
            continue
        for lane in range(1, lanes_per_edge):
            laneID = edge + "_" + str(lane)
            lanes.append(laneID)
    return lanes
def batch_matcher(v1, v2):
    ranges = [(0, 288), (288, 576), (576, 864), (864, 1152), (1152, 1440), (1440, 1728), (1728, 2016),\
              (2016, 2304), (2304, 2592), (2592, 2880), (2880, 3168), (3168, 3456), (3456, 3744), (3744, 4032), (4032, 4320)]
    v1_ind = int(v1.split(".")[1])
    v2_ind = int(v2.split('.')[1])
    #for el in ranges:
    v1_batch = [el for el in ranges if v1_ind in range(el[0],el[1])]
    v2_batch = [el for el in ranges if v2_ind in range(el[0],el[1])]
    return v1_batch == v2_batch

def sorted_planes(lane_vehicles, lane):
    """
    The function takes in the lane and the vehs on the lane and returns
    plane class objects.
    to ensure that vehicles on the same lane are in the same platoon,
    the function checks the distance of the veh to the leader and also 
    the route of the vehicle.
    This function is capable of generating primary and secondary platoons
    """
    planes = []
    primary_plane = []
    secondary_plane1 = []
    secondary_plane2 = []
    secondary_plane3 = []
    leader_route = traci.vehicle.getRoute(lane_vehicles[0])
    plength = len(lane_vehicles)
    for vehicle in lane_vehicles:
        if traci.vehicle.getRoute(vehicle) == leader_route and get_distance(vehicle, lane_vehicles[0]) < (
                240 + 200) and batch_matcher(lane_vehicles[0], vehicle): 
            primary_plane.append(vehicle)
        else:
            secondary_plane1.append(vehicle) 
    for veh in secondary_plane1:
        if traci.vehicle.getRoute(veh) != traci.vehicle.getRoute(secondary_plane1[0]) or get_distance(veh, secondary_plane1[0]) > (N_VEHICLES*LENGTH) + (N_VEHICLES-1)*DISTANCE + 100:
            secondary_plane1.pop(secondary_plane1.index(veh))
            secondary_plane2.append(veh)
    for veh in secondary_plane2:
        if traci.vehicle.getRoute(veh) != traci.vehicle.getRoute(secondary_plane2[0]) or get_distance(veh, secondary_plane2[0]) > (N_VEHICLES*LENGTH) + (N_VEHICLES-1)*DISTANCE + 100:
            secondary_plane2.pop(secondary_plane2.index(veh))
            secondary_plane3.append(veh)
    ps_planes = [primary_plane, secondary_plane1, secondary_plane2, secondary_plane3]
    all_planes = []
    for item in ps_planes:
        item_planes_with_empties = [(item[plength * i: plength * i + plength]) for i in range(plength)]
        item_planes = [plane for plane in item_planes_with_empties if plane != []]
        for plane in item_planes:
            planes.append(planers.Plane(
                lane, plane))
    return planes

def get_planar_dist(v1,v2):
    cur_edge1= traci.vehicle.getRoadID(v1)
    cur_edge2= traci.vehicle.getRoadID(v2)
    lane_pos1= traci.vehicle.getLanePosition(v1)
    lane_pos2= traci.vehicle.getLanePosition(v2)
    # Collapse ring to single edge by adding lens of past edges to edge pos traci provides
    cum_len_tups = [('p0',0),('n1',1000),('p2',2000),('n3',12000),('p4',13000),('p5',14000),('n6',15000),('p7',16000),('n8',26000),\
                        ('p9',27000),('p10',28000),('n11',29000),('p12',30000),('n13',40000),('p14',41000),('p15',42000),('n16',43000)\
                        ,('p17',44000),('n18',54000),('p19',55000)]
    # if on source edges, actual projection onto adjacent highway edge is performed
    # len of source edges 400+540, difference 60 compared to adjacent edge
    source_cum_len_tups = [('source0',60),('s0',460),('source1',14060),('s1',14460),('source2',28060),('s2',28460),('source3',42060),('s3',42460)]
    all_cum_len_tups = cum_len_tups + source_cum_len_tups
    cum_dist1 = [item[1] for item in all_cum_len_tups if item[0]==cur_edge1]
    cum_dist2 = [item[1] for item in all_cum_len_tups if item[0]==cur_edge2]
    planar_dist1 = lane_pos1 + cum_dist1[0]
    planar_dist2 = lane_pos2 + cum_dist2[0]
    planar_dist = math.sqrt((planar_dist1 - planar_dist2)**2)
    return planar_dist

def arrived_vehz(all_arrived):
    arr_this_step= traci.simulation.getArrivedIDList()
    for veh in arr_this_step:
        all_arrived.append(veh)
    return all_arrived

def get_leaders(batchnum, all_arrived):
    leaderz=['v.'+str(i*24) for i in range(batchnum)]
    leaderz =[leader for leader in leaderz if leader not in all_arrived]
    leaders=[]
    for leader in leaderz:
        try:
            traci.vehicle.getLanePosition(leader)
        except:
            continue
        else:
            leaders.append(leader)

    return leaders

def change_lanes(dest_lane):
    for i in range(len(dest_lane)):
        for vehicle in dest_lane[i]:
            change_lane(vehicle, i + 1)
def sorted_lane_vehs(veh_of_interest):
    swap_edges = ["p2", "p7", "p12", "p17"]
    ## might need to add other edges here to consider planes that are not on swap lanes
    lanes = []
    sorted_swap_vehs = {}
    for edge in swap_edges:
        for i in range(1, 4):
            lanes.append(edge + "_" + str(i))
    for lane in lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
        if vehicles == []:
            continue
        vehicles = [veh for veh in vehicles if batch_matcher(veh_of_interest,veh)==True]
        if vehicles == []:
            continue
        sorted_swap_vehs[lane] = vehicles
    return sorted_swap_vehs
def sort_flag_vehs():
        lanes = []
        sorted_flag_vehs = {}
        flag_edges = ["n3", "n8", "n13", "n18"]
        for edge in flag_edges:
            for i in range(1, 4):
                lanes.append(edge + "_" + str(i))
        for lane in lanes:
            vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
            if vehicles == []:
                continue
            sorted_flag_vehs[lane] = vehicles

        return sorted_flag_vehs

def remove_parked(removed_vehs):
    off_ramp_edges = ['n3','n8','n13', 'n18']
    for item in off_ramp_edges:
        if traci.edge.getLastStepHaltingNumber(item):
            parking_vehs=  [veh for veh in traci.edge.getLastStepVehicleIDs(item)[::-1] if traci.vehicle.getSpeed(veh)<= 0.2]
            for veh in parking_vehs:
                traci.vehicle.remove(veh, REMOVE_PARKING)
                print(f'veh {veh} removed for parking')
                removed_vehs.append(veh)

def get_leaders_pos(batchnum, all_arrived):
    cur_time = traci.simulation.getCurrentTime()
    cum_len_tups = [('p0',0),('n1',1000),('p2',2000),('n3',12000),('p4',13000),('p5',14000),('n6',15000),('p7',16000),('n8',26000),('p9',27000),\
                    ('p10',28000),('n11',29000),('p12',30000),('n13',40000),('p14',41000),('p15',42000),('n16',43000),('p17',44000),('n18',54000),('p19',55000)]
    source_cum_len_tups = [('source0',60),('s0',460),('source1',14060),('s1',14460),('source2',28060),('s2',28460),('source3',42060),('s3',42460)]
    all_cum_len_tups = cum_len_tups + source_cum_len_tups
    leader_dis_tups= []
    leaders = get_leaders(batchnum, all_arrived)
    for leader in leaders:
        try:
            leader_cur_edge= traci.vehicle.getRoadID(leader)
            leader_lane_pos= traci.vehicle.getLanePosition(leader)
        except:
            try:
                traci.vehicle.getLanePosition(leader)
            except:
                print("Veh doesn't exist")
            else:
                list_of_leaders.remove(leader)
        else:
            if not leader_cur_edge.startswith(':') and not leader_cur_edge.startswith('e'):

                leader_cum_dist = [item[1] for item in all_cum_len_tups if item[0]==leader_cur_edge]
                leader_planar_dist = leader_lane_pos + leader_cum_dist[0]
                leader_dis_tups.append([cur_time,leader,leader_planar_dist])
            else:
                continue
    return leader_dis_tups

def insert_switch(batchnum):
    """
    Platoon trajectory departure check
    The trajectory of an platoon at a given source is compared to the
    trajectories of of all other platoons in the network. The trajectories of both
    the leader of the platoon yet to be departed and existing platoon leaders are buffered by a skirt to 
    take into into account all trajectories of the individual vehicles in a platoon plus a safety
    margin. If no interceptions exists between the trajectories, the platoon is safet to depart, otherwise
    the departure is postponed and re-evaluated at the next evaluation time step.
    """
    leaders = get_leaders(batchnum, all_arrived)
    leaders_dis_tups = get_leaders_pos(batchnum, all_arrived)
    lane_indices = ['1','2','3']
    new_leader_ini_planar_dis = 160 + N_VEHICLES*LENGTH + (N_VEHICLES-1)*DISTANCE
    new_leader_fin_planar_dis1 = 23+540+1000+10000+1000 # total distance from where leader starts to exit ramp
    new_leader_fin_planar_dis2 = 23 + 540 + 1000 + 10000 + 1000 + 1000 + 1000 + 10000 + 1000
    new_leader_fin_planar_dis3 = 23 + 540 + 1000 + 10000 + 1000 + 1000 + 1000 + 10000 + 1000 + 1000 + 1000 + 1000 + 10000 + 1000
    fin_planar_diss=[new_leader_fin_planar_dis1,new_leader_fin_planar_dis2,new_leader_fin_planar_dis3]
    new_leader_vel1 = PSPEED + 4
    new_leader_vel2 = PSPEED + 7
    new_leader_vel3 = PSPEED + 10
    cur_time = traci.simulation.getCurrentTime()
    lane_flags = []
    for index in lane_indices:
        lane_index = index
        if index == '1':
            new_leader_arrival_time = math.sqrt((new_leader_fin_planar_dis1 - new_leader_ini_planar_dis)**2)/(new_leader_vel1)
            new_leader_time_array = np.linspace(cur_time, new_leader_arrival_time, num=100)
            new_leader_pos_array = np.linspace(new_leader_ini_planar_dis, new_leader_fin_planar_dis1, num=100)
            new_platoon_trajectory = LineString(zip(new_leader_time_array, new_leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
            all_interx=[]
            for leader in leaders:
                leader_lane=traci.vehicle.getLaneID(leader)
                leader_lane_index= leader_lane.split('_')[1]
                leader_final_planar_dis= fin_planar_diss[int(leader_lane_index)-1]
                leader_cur_planar_pos= [tup[2] for tup in leaders_dis_tups if tup[1]==leader]
                leader_arrival_time = math.sqrt((leader_final_planar_dis-leader_cur_planar_pos[0])**2)/traci.vehicle.getSpeed(leader)
                leader_time_array= np.linspace(cur_time,leader_arrival_time, num=100)
                leader_pos_array = np.linspace(leader_cur_planar_pos, leader_final_planar_dis, num=100) 
                leader_platoon_trajectory= LineString(zip(leader_time_array, leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
                interx=leader_platoon_trajectory.intersection(new_platoon_trajectory)
                if not interx.is_empty:
                    all_interx.append((leader,interx))
                else:
                    continue
            if all_interx == []:
                lane1_flag = True
            else:
                lane1_flag=False

            lane_flags.append(lane1_flag)
        if index == '2':
            new_leader_arrival_time = math.sqrt((new_leader_fin_planar_dis2 - new_leader_ini_planar_dis)**2)/(new_leader_vel2)
            new_leader_time_array = np.linspace(cur_time, new_leader_arrival_time, num=100)
            new_leader_pos_array = np.linspace(new_leader_ini_planar_dis, new_leader_fin_planar_dis2, num=100)
            new_platoon_trajectory = LineString(zip(new_leader_time_array, new_leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
            all_interx=[]
            for leader in leaders:
                leader_lane=traci.vehicle.getLaneID(leader)
                leader_lane_index= leader_lane.split('_')[1]
                leader_final_planar_dis= fin_planar_diss[int(leader_lane_index)-1]
                leader_cur_planar_pos= [tup[2] for tup in leaders_dis_tups if tup[1]==leader]
                leader_arrival_time = math.sqrt((leader_final_planar_dis-leader_cur_planar_pos[0])**2)/traci.vehicle.getSpeed(leader)
                leader_time_array= np.linspace(cur_time,leader_arrival_time, num=100)
                leader_pos_array = np.linspace(leader_cur_planar_pos, leader_final_planar_dis, num=100) 
                leader_platoon_trajectory= LineString(zip(leader_time_array, leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
                interx=leader_platoon_trajectory.intersection(new_platoon_trajectory)
                if not interx.is_empty:
                    all_interx.append((leader,interx))
                else:
                    continue
            if all_interx == []:
                lane2_flag = True
            else:
                lane2_flag=False
            lane_flags.append(lane2_flag)
        if index == '3':
            new_leader_arrival_time = math.sqrt((new_leader_fin_planar_dis3 - new_leader_ini_planar_dis)**2)/(new_leader_vel3)
            new_leader_time_array = np.linspace(cur_time, new_leader_arrival_time, num=100)
            new_leader_pos_array = np.linspace(new_leader_ini_planar_dis, new_leader_fin_planar_dis3, num=100)
            new_platoon_trajectory = LineString(zip(new_leader_time_array, new_leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
            all_interx=[]
            for leader in leaders:
                leader_lane=traci.vehicle.getLaneID(leader)
                leader_lane_index= leader_lane.split('_')[1]
                leader_final_planar_dis= fin_planar_diss[int(leader_lane_index)-1]
                leader_cur_planar_pos= [tup[2] for tup in leaders_dis_tups if tup[1]==leader]
                leader_arrival_time = math.sqrt((leader_final_planar_dis-leader_cur_planar_pos[0])**2)/traci.vehicle.getSpeed(leader)
                leader_time_array= np.linspace(cur_time,leader_arrival_time, num=100)
                leader_pos_array = np.linspace(leader_cur_planar_pos, leader_final_planar_dis, num=100) 
                leader_platoon_trajectory= LineString(zip(leader_time_array, leader_pos_array)).buffer(N_VEHICLES*LENGTH+(N_VEHICLES-1)*DISTANCE)
                interx=leader_platoon_trajectory.intersection(new_platoon_trajectory)
                if not interx.is_empty:
                    all_interx.append((leader,interx))
                else:
                    continue
            if all_interx == []:
                lane3_flag = True
            else:
                lane3_flag = False
            lane_flags.append(lane3_flag)
    return lane_flags

def upstream_check(batchnum, lane, all_arrived):
    """
    This method searches upstream of the source where a platoon is to be departed 
    to find oncomming platoons whose time of arrival at the on-ramp may coincide with 
    departing platoon. The arrival time of the departing platoon is compared with that 
    of oncoming platoons. If the arrival times of the platoons are separated by enough 
    headway a green flag is raised, otherwise a red flag is raised
    """
    leaders = get_leaders(batchnum, all_arrived)
    corner_edges = ['p0', 'p19']
    corner_leaders = []
    source_edge_len = 400
    buffer_time = 20.0
    ep_edge ='n1'
    ep_loc =0.0
    new_leader_dis_ep = source_edge_len - (100 + N_VEHICLES*LENGTH + (N_VEHICLES-1)*DISTANCE ) +540
    new_leader_ep_arr_time1 = new_leader_dis_ep/(PSPEED + 4)
    new_leader_ep_arr_time2 = new_leader_dis_ep/(PSPEED + 7)
    new_leader_ep_arr_time3 = new_leader_dis_ep/(PSPEED + 10)
    new_leader_ep_arr_times =[new_leader_ep_arr_time1,new_leader_ep_arr_time2, new_leader_ep_arr_time3]
    switch_states=[]
    if lane==1:
        lanes_of_int = [1]
    elif lane==2:
        lanes_of_int=[1,2]
    else:
        lanes_of_int=[1,2,3]

    for leader in leaders:
        if traci.vehicle.getRoadID(leader) in corner_edges and int(traci.vehicle.getLaneID(leader).split('_')[1]) in lanes_of_int:
            corner_leaders.append(leader)
    if corner_leaders!=[]:
        for leader in corner_leaders:
            leader_cur_dis_ep = traci.vehicle.getDrivingDistance(leader, ep_edge, ep_loc, laneIndex=lane)
            leader_cur_speed = traci.vehicle.getSpeed(leader)
            leader_ep_arr_time = leader_cur_dis_ep/leader_cur_speed
            new_leader_ep_arr_time = new_leader_ep_arr_times[lane-1]
            if new_leader_ep_arr_time < leader_ep_arr_time - buffer_time: # If condition is satisfied new leader safely passes entry point
                switch='GREEN'
                switch_states.append((leader, switch, lane))              # at least a 100s before the platoon approaching
            else:
                switch = 'RED'
                switch_states.append((leader, switch, lane))
        return switch_states
    else:
        switch='GREEN'
        leader = 'None'
        switch_states.append((leader,switch, lane))
        return switch_states

def downstream_check(batchnum,lane, all_arrived):
    """
    This method searches downstream of the source where a platoon is to be departed 
    to find oncomming platoons whose time of arrival at the next lane-change station may coincide with 
    departing platoon. The arrival time of the departing platoon is compared with that 
    of oncoming platoons. If the arrival times of the platoons are separated by enough 
    headway a green flag is raised, otherwise a red flag is raised
    """
    leaders = get_leaders(batchnum,all_arrived)
    downstream_edges = ['p19','p0','s0','n1', 'p2', 'n3']
    downstream_leaders = []
    new_leader_vel1 = PSPEED + 4
    new_leader_vel2 = PSPEED + 7
    new_leader_vel3 = PSPEED + 10
    lc_edge='n3'
    lc_loc= 430.0
    new_leader_vels =[new_leader_vel1, new_leader_vel2, new_leader_vel3]
    new_leader_cur_pos = (100 + N_VEHICLES * LENGTH + (N_VEHICLES - 1) * DISTANCE)
    new_leader_dis_lcs = (400-new_leader_cur_pos) +540+12000.0+430.0
    buffer_time = 50.0
    switch_states =[]
    if lane == 1:
        lanes_of_int = [1,2]
    elif lane == 2:
        lanes_of_int = [1, 3]
    else:
        lanes_of_int = [2]
    for leader in leaders:
        if traci.vehicle.getRoadID(leader) in downstream_edges and int(traci.vehicle.getLaneID(leader).split('_')[1]) in lanes_of_int:
            downstream_leaders.append(leader)
    if downstream_leaders!=[]:
        for leader in downstream_leaders:
            #get leader's time to lane change flag
            leader_dis_lc = traci.vehicle.getDrivingDistance(leader, lc_edge, lc_loc, laneIndex=lane)
            leader_speed = traci.vehicle.getSpeed(leader)
            leader_time_lcs = leader_dis_lc/leader_speed
            # calculate imaginary new leader time to lane change station
            new_leader_vel= new_leader_vels[lane-1]
            new_leader_time_lcs=new_leader_dis_lcs/new_leader_vel
            if new_leader_time_lcs > leader_time_lcs + buffer_time:
                switch = 'GREEN'
                switch_states.append((leader, switch, lane))
            else:
                switch= 'RED'
                switch_states.append((leader, switch, lane))
        return switch_states
    else:
        switch='GREEN'
        leader = 'None'
        switch_states.append((leader,switch, lane))
        return switch_states

def switching_logic(switches):
    """
    This method decyphers the switch_state provided by the upstream 
    and downstream checks to evaluate the different flags in the two checks
    A single red flag either upstream or downstream should negate all other green
    flags for safety reasons.
    """
    lane1_checks = switches[:2]
    lane2_checks = switches[2:4]
    lane3_checks = switches[4::]
    lane1_upstream= lane1_checks[0]
    lane1_downstream = lane1_checks[1]
    lane1_up_reds =[]
    lane1_down_reds =[]
    for item in lane1_upstream:
        if item[1] == 'RED':
            lane1_up_reds.append(1)
    if lane1_up_reds == []:
        for item in lane1_downstream:
            if item[1] == 'RED':
                lane1_down_reds.append(1)
        if lane1_down_reds==[]:
            lane1_switch = 'GREEN'
        else:
            lane1_switch = 'RED'
    else:
        lane1_switch = 'RED'
    lane2_upstream= lane2_checks[0]
    lane2_downstream = lane2_checks[1]
    lane2_up_reds =[]
    lane2_down_reds =[]
    for item in lane2_upstream:
        if item[1] == 'RED':
            lane2_up_reds.append(1)
    if lane2_up_reds == []:
        for item in lane2_downstream:
            if item[1] == 'RED':
                lane2_down_reds.append(1)
        if lane2_down_reds==[]:
            lane2_switch = 'GREEN'
        else:
            lane2_switch = 'RED'
    else:
        lane2_switch = 'RED'

    lane3_upstream= lane3_checks[0]
    lane3_downstream = lane3_checks[1]
    lane3_up_reds =[]
    lane3_down_reds =[]
    for item in lane3_upstream:
        if item[1] == 'RED':
            lane3_up_reds.append(1)
    if lane3_up_reds == []:
        for item in lane3_downstream:
            if item[1] == 'RED':
                lane3_down_reds.append(1)
        if lane3_down_reds==[]:
            lane3_switch = 'GREEN'
        else:
            lane3_switch = 'RED'
    else:
        lane3_switch = 'RED'
    # # # Raw values
    priority_switching = [lane1_switch, lane2_switch, lane3_switch]
    return priority_switching

def main(real_engine, setter=None, demo_mode=False):
    global genStep # 
    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    batch_num = 0
    source_edges = ['source0', 'source1', 'source2', 'source3']
    removed_vehs = []
    all_arrives = []
    all_arrived = []
    edge_filter, vtype_filter = validate_params(
        edge_filter=PLAT_EDGES, vtype_filter=["vtypeauto"])
    pstate = INSERT
    while running(demo_mode, step, 2200):
        if demo_mode and step == 2200:
            start_sumo("cfg/freeway.sumo.cfg", False)
            step = 0
        print("step is : {}".format(step))
        print("Current time is :{}".format(traci.simulation.getCurrentTime()))
        print("pstate is : {}".format(pstate))
        if pstate == INSERT:
            add_vehicles(N_VEHICLES_GEN, batch_num, list_of_leaders, fromEdge=source_edges[0], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, list_of_leaders, fromEdge=source_edges[1], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, list_of_leaders, fromEdge=source_edges[2], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, list_of_leaders, fromEdge=source_edges[3], platoon_len=24, real_engine=False)
            batch_num = batch_num + 3
            traci.gui.setZoom("View #0", 4500)
            all_arrived=arrived_vehz(all_arrives)
            all_arrives=all_arrived
            traci.simulationStep()
            pstate = PLATOONING
            genStep = step
        if pstate == INSERT2:
            inserted = add_vehiclez(N_VEHICLES_GEN, batch_num, list_of_leaders, priorityswitches, fromEdge=source_edges[0], platoon_len=24,
                         real_engine=False)
            batch_num = batch_num + inserted
            inserted = add_vehiclez(N_VEHICLES_GEN, batch_num, list_of_leaders, priorityswitches, fromEdge=source_edges[1], platoon_len=24,
                         real_engine=False)
            batch_num = batch_num + inserted
            inserted = add_vehiclez(N_VEHICLES_GEN, batch_num, list_of_leaders, priorityswitches, fromEdge=source_edges[2], platoon_len=24,
                         real_engine=False)
            batch_num = batch_num + inserted
            inserted = add_vehiclez(N_VEHICLES_GEN, batch_num, list_of_leaders, priorityswitches, fromEdge=source_edges[3], platoon_len=24,
                         real_engine=False)
            batch_num = batch_num + inserted
            traci.gui.setZoom("View #0", 4500)
            all_arrived=arrived_vehz(all_arrives)
            all_arrives=all_arrived
            traci.simulationStep()
            topology = {}
            pstate = PLATOONING
            genStep = step
        if pstate == PLATOONING and step >= genStep + 110:
            switches = []
            for lane in range(1,4):
                upstreamcheck = upstream_check(batch_num,lane, all_arrived)
                downstreamcheck = downstream_check(batch_num, lane, all_arrived)
                switches.append(upstreamcheck)
                switches.append(downstreamcheck)
            priorityswitches = switching_logic(switches)
            if 'GREEN' in priorityswitches:
                pstate=INSERT2
        if step > genStep + 1 and pstate == PLATOONING:
            flag_planes = []
            lanes = lane_gen()
            for lane in lanes:
                if not traci.lane.getLastStepVehicleIDs(lane):
                    continue
                lane_vehicles = [veh for veh in traci.lane.getLastStepVehicleIDs(lane)[::-1] if veh not in removed_vehs]
                planes = sorted_planes(lane_vehicles, lane)
                for plane in planes:
                    if plane.near_flag():
                        flag_planes.append(plane)
                    teleported_vehicles = traci.simulation.getEndingTeleportIDList()
                    for vehicle in teleported_vehicles:
                        try:
                            traci.vehicle.remove(vehicle, REMOVE_PARKING)
                        except:
                            print(f"Veh {vehicle} has been removed already")
                        else:
                            print(f"Veh {vehicle} has been removed")
                            removed_vehs.append(vehicle)
                    topology = plane.topo_contsructor(removed_vehs)
                    topology = plane.pla_speed_spacing(topology)
                    communicate(topology)
                    all_arrived=arrived_vehz(all_arrives)
                    all_arrives = all_arrived
                    for plane in flag_planes:
                        flag_n_poi_index = plane.look_for_flags(pois, step)
                        if flag_n_poi_index[0] == True:
                            plane.move_to_next_best_lane(step, flag_n_poi_index)
                            plane.set_arrived_free()
                    traci.simulationStep()
        remove_parked(removed_vehs)
        teleported_vehicles = traci.simulation.getEndingTeleportIDList()
        for vehicle in teleported_vehicles:
            try:
                traci.vehicle.remove(vehicle, REMOVE_PARKING)
            except:
                print(f"Veh {vehicle} has been removed already")
            else:
                print(f"Veh {vehicle} has been removed")
                removed_vehs.append(vehicle)
        step += 1
    traci.close()
if __name__ == "__main__":
    main(True, True)
