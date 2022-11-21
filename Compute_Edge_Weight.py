import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
import matplotlib.pyplot as plt 
from tooluse import compute_left_near_veh_region,compute_right_near_veh_region,check_dead,all_path,get_regions_crowds_information,compute_front_veh_region,compute_far_veh_region,compute_near_veh_region,summax,sigmoid,softmax,NormalizeData,negative_vector,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from numpy import random
def degree_of_passage_w(ped_ev,region,current_node):

	_region_node_dict = ped_ev.graph._region_node_dict
	node = _region_node_dict[region]
	gap_distance = 0
	
	for dic_layer in ped_ev.graph.node_gap:
		if node in dic_layer:
			gap_distance = dic_layer[node]
			break

	cw = 2*ped_ev.pedestrain_r + 1
	dop = max((gap_distance - cw)/cw,0.0)

	return dop
def travel_length_w(ped_ev,G,ped_p,front_vec,right_vec,region,goal_sidewalk_region):	
	#destination_point = None
	ped_p = carla.Location(ped_p.x,ped_p.y,0.0)
	travel_length = 0
	mid_point = get_mid_point(region)
	p1,p2,p3,p4 = region[0],region[1],region[2],region[3]
	p1,p2,p3,p4 = carla.Location(p1.x,p1.y,0.0),carla.Location(p2.x,p2.y,0.0),carla.Location(p3.x,p3.y,0.0),carla.Location(p4.x,p4.y,0.0)
	h = ped_ev.graph.layers_height
	rolling = True
	_,p = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[mid_point.x,mid_point.y,mid_point.x+right_vec.x,mid_point.y+right_vec.y])	
	if isInRect(p1,p2,p3,p4,carla.Location(p[0],p[1],0.0)):
		rolling = False
		
	bP = carla.Location(p[0],p[1],0.0) - front_vec*(h/2)

	if rolling:
		dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
		p = [p1,p2,p3,p4][np.argmin(dis_L)]
		travel_length = ped_p.distance(carla.Location(p.x,p.y,0.0))
	else:
		travel_length = ped_p.distance(carla.Location(bP.x,bP.y,0.0))

	dead = check_dead(ped_ev,G,region,front_vec,right_vec,0,goal_sidewalk_region,[region])
	#travel_length = sigmoid(travel_length,True)
	return travel_length,dead,rolling	
def decayed_rate_w(ped_ev,c,region):
	#Leader speed - follower speed
	delta_Vm = 0
	_region_node_dict = ped_ev.graph._region_node_dict
	node = _region_node_dict[region]
	Leader_speed = ped_ev.veh_env.velo_i[node[0]] if node[0] != -1 else 0
	Follower_speed = ped_ev.veh_env.velo_i[node[1]] if node[1] != -1 else 0
	delta_Vm = Leader_speed - Follower_speed
	return delta_Vm	
def region_flow_Occupancy_rate_w(ped_ev,ped_p,right_vec,front_vec,n,i):
	region_flow = 0
	Occupancy_rate = 0
	_region_node_dict = ped_ev.graph._region_node_dict
	node = _region_node_dict[n]
	
	h = ped_ev.graph.layers_height
	w = 0
	for dic_layer in ped_ev.graph.node_gap:
		if node in dic_layer:
			w = dic_layer[node]
			break

	ped_type1_in_region = ped_ev.ped_type1_in_region
	ped_type2_in_region = ped_ev.ped_type2_in_region
	
	if ped_ev.ped_type[i] == 1:
		diff_dir_ped_id,same_dir_ped_id = ped_type2_in_region[n],ped_type1_in_region[n]
	else:
		diff_dir_ped_id,same_dir_ped_id = ped_type1_in_region[n],ped_type2_in_region[n]

	num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)

	if num_not_same_direction + num_same_direction == 0:
		region_flow = 0.5
		Occupancy_rate = 0.0
	else:
		region_flow = num_same_direction/(num_not_same_direction + num_same_direction)
		Occupancy_rate = (2*ped_ev.pedestrain_r * 2*ped_ev.pedestrain_r * (num_not_same_direction + num_same_direction))/(h*w) if (h*w) > 0 else sys.maxsize

	return region_flow,Occupancy_rate
def compute_edge_weight(ped_ev,G,next_regions,ped_p,i,front_vec,right_vec,current_region,goal_sidewalk_region):

	degree_of_passages = []
	travel_lengths = []
	decayed_rate = []
	Occupancy_rate = []
	region_flow = []
	dead_road = []
	
	Region_distances = []
	search_gap = []
	
	for c,region in enumerate(next_regions):
		#1
		degree_of_passage = degree_of_passage_w(ped_ev,region,current_region)
		degree_of_passages.append(degree_of_passage)
		
		#2
		travel_length,dead,rolling = travel_length_w(ped_ev,G,ped_p,front_vec,right_vec,region,goal_sidewalk_region)
		travel_lengths.append(travel_length)
		Region_distances.append(travel_length)
		dead_road.append(dead)
		search_gap.append(rolling)
		
		#3
		decayed = decayed_rate_w(ped_ev,c,region)
		decayed_rate.append(decayed)
		
		#4
		flow,Occupancy = region_flow_Occupancy_rate_w(ped_ev,ped_p,right_vec,front_vec,region,i)
		Occupancy_rate.append(Occupancy)
		region_flow.append(flow)
	
	#travel_lengths = summax(travel_lengths,True) if sum(travel_lengths) != 0 else [0 for i in range(len(travel_lengths))]		
	#print(decayed_rate)
	
	mindecay = min(decayed_rate)
	decayed_rate = [rate + abs(mindecay) for rate in decayed_rate] if mindecay < 0 else decayed_rate
	degree_of_passages = summax(degree_of_passages,False) if sum(degree_of_passages) != 0 else [0 for i in range(len(degree_of_passages))]
	travel_lengths = summax(travel_lengths,True) if sum(travel_lengths) != 0 else [0 for i in range(len(travel_lengths))]	
	decayed_rate = summax(decayed_rate,False) if sum(decayed_rate) != 0 else [0 for i in range(len(decayed_rate))]		
	region_flow = summax(region_flow,False) if sum(region_flow) != 0 else [0 for i in range(len(region_flow))]	
	Occupancy_rate = summax(Occupancy_rate,True) if sum(Occupancy_rate) != 0 else [0 for i in range(len(Occupancy_rate))]	

	return degree_of_passages,travel_lengths,decayed_rate,Occupancy_rate,region_flow,dead_road,Region_distances,search_gap