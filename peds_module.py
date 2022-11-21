import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
import matplotlib.pyplot as plt 
from tooluse import vecs2angle,nearest_next_nodes,draw_circles,get_Node_region_Left_Right_mid,get_front_region,regions_distance,get_front_region,Compute_grid_value2,check_front_move_around,get_len,get_Node_Width_Height,get_next_grid,get_Grid,Compute_grid_value,get_regions_crowds_information,get_LFR_grid,get_Left_Right_Front_Local_region,NormalizeData,negative_vector,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from numpy import random
from Compute_Edge_Weight import compute_edge_weight

def init(ped_ev):
	ped_ev.block = np.zeros((ped_ev.size()))
	ped_ev.left_block = np.zeros((ped_ev.size()))
	ped_ev.right_block = np.zeros((ped_ev.size()))
	ped_ev.rear_block = np.zeros((ped_ev.size()))
	ped_ev.ped_front_block = np.zeros((ped_ev.size()))

	ped_ev.gate_close = np.zeros((ped_ev.size(),2))
	ped_ev.front_block_point = np.zeros((ped_ev.size(),2))
	ped_ev.left_block_point = np.zeros((ped_ev.size(),2))
	ped_ev.right_block_point = np.zeros((ped_ev.size(),2))
	ped_ev.ped_front_block_point = np.zeros((ped_ev.size(),2))
	ped_ev.rear_block_point = np.zeros((ped_ev.size(),2))  

	ped_ev.left_boundry = np.zeros((ped_ev.size(),2))
	ped_ev.right_boundry = np.zeros((ped_ev.size(),2))
	ped_ev.rear_boundry = np.zeros((ped_ev.size(),2))

def check_state(ped_ev,ped_p,i):	
	
	crossing_state = 1	
	in_layer_id = -1

	for item in ped_ev.ped_side_walk:
		if isInRect(item[0][0],item[0][1],item[0][2],item[0][3],ped_p) and item[1] == ped_ev.ped_type[i]:
			crossing_state = 0
			break
		elif isInRect(item[0][0],item[0][1],item[0][2],item[0][3],ped_p) and item[1] != ped_ev.ped_type[i]:
			crossing_state = 2
			break
			
	src_sidewalk_region = None
	goal_sidewalk_region = None
	
	for item in ped_ev.ped_side_walk:
		if item[1] != ped_ev.ped_type[i]:
			goal_sidewalk_region = item[0]
		if item[1] == ped_ev.ped_type[i]:
			src_sidewalk_region = item[0]
			
	for layer_id,layer_rect in enumerate(ped_ev.graph.layers_rect):
		if isInRect(layer_rect[0],layer_rect[1],layer_rect[2],layer_rect[3],ped_p):
			in_layer_id = layer_id
			break
	
	return crossing_state,goal_sidewalk_region,src_sidewalk_region,in_layer_id
	
def Caculate_and_Get_target_Region(ped_ev,i,G,next_regions,current_loc,current_region,front_vec,right_vec,goal_sidewalk_region):
	
	index,Region_values = -1,[]
	if next_regions == None:
		return None,0
	if len(next_regions) > 1:
		degree_of_passages,travel_lengths,decayed_rate,Occupancy_rate,region_flow,dead_road,Region_distances,search_gap = compute_edge_weight(ped_ev,G,next_regions,current_loc,i,front_vec,right_vec,current_region,goal_sidewalk_region)
		target_Region,index,Region_values = ped_ev.get_Next_target(next_regions,degree_of_passages,travel_lengths,decayed_rate,Occupancy_rate,region_flow,dead_road,i,search_gap)
	elif len(next_regions) == 1:
		target_Region = next_regions[0]
	else:
		target_Region = None
	
	Region_value = 0
	if index != -1:
		Region_value = Region_values[index]
		
	return target_Region,Region_value
	
def gap_acceptance(ped_ev,oncoming_vehicle,front_vec,right_vec,ped_p,i):
	should_stop = False
	if oncoming_vehicle != -1:
		#ped_ev.world.draw_line(ped_p,ped_ev.vehicles[oncoming_vehicle].get_location(),0.2,carla.Color(255,255,0))
		l = ped_ev.vehicles[oncoming_vehicle].get_location()
		l.z = 2.0
		#ped_ev.world.draw_point(l,0.2,carla.Color(255,255,0))
		n = ped_p-ped_ev.vehicles[oncoming_vehicle].get_location()
		n.z = 0
		#self.Invoveh[i] = np.array([n.x,n.y])
		
		ovl = ped_ev.vehicles[oncoming_vehicle].get_location()+ped_ev.vehicles[oncoming_vehicle].get_transform().get_forward_vector()*ped_ev.veh_rear[oncoming_vehicle]
		dv = abs(ovl.x - ped_p.x)
		#self.world.draw_line(ovl+dv*self.vehicles[oncoming_vehicle].get_transform().get_forward_vector(),ovl,0.2,carla.Color(245,0,0))
		tv = dv/ped_ev.veh_env.velo_i[oncoming_vehicle] if ped_ev.veh_env.velo_i[oncoming_vehicle] > 0 else sys.maxsize
		vehicleInfrontpoint = ped_ev.vehicles[oncoming_vehicle].get_location() + front_vec * ped_ev.veh_raduis[oncoming_vehicle]
		
		dp = abs(ped_p.y - vehicleInfrontpoint.y)
		tp = dp/ped_ev.max_speed_multiplier[i]
		
		#print(self.veh_env.velo_i[oncoming_vehicle])
		
		#self.world.draw_line(carla.Location(ped_p.x,ped_p.y,0.2)+front_vec*dp,carla.Location(ped_p.x,ped_p.y,0.2),0.2,carla.Color(245,0,0))
		#self.world.draw_point(ovl,0.2,carla.Color(245,0,0))
		
		if tp >= tv:
			should_stop = True
		
		if dp >= 5:
			should_stop = False

	return should_stop

def getfourpoint(ped_ev,ped_p,current_region,front_vec,right_vec):
	
	ped_p = carla.Location(ped_p.x,ped_p.y,0.0)
	front_point,back_point,right_point,left_point = None,None,None,None
	dis2front,dis2back,dis2right,dis2left = -1,-1,-1,-1
		
	if current_region != None:
	
		p1,p2,p3,p4 = current_region[0],current_region[1],current_region[2],current_region[3]
		mid_point = get_mid_point(current_region)
		h = ped_ev.graph.layers_height
		
		_region_node_dict = ped_ev.graph._region_node_dict
		node = _region_node_dict[current_region]
		w = 0
		for dic_layer in ped_ev.graph.node_gap:
			if node in dic_layer:
				w = dic_layer[node]
				break 

		right_front,right_back = mid_point + (w/2)*right_vec+(h/2)*front_vec,mid_point + (w/2)*right_vec-(h/2)*front_vec
		left_front,left_back = mid_point - (w/2)*right_vec+(h/2)*front_vec,mid_point - (w/2)*right_vec-(h/2)*front_vec
		
		_,front_point = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[right_front.x,right_front.y,left_front.x,left_front.y])
		_,back_point = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[right_back.x,right_back.y,left_back.x,left_back.y])
		
		_,right_point = cross_point([ped_p.x,ped_p.y,ped_p.x+right_vec.x,ped_p.y+right_vec.y],[right_front.x,right_front.y,right_back.x,right_back.y])
		_,left_point = cross_point([ped_p.x,ped_p.y,ped_p.x+right_vec.x,ped_p.y+right_vec.y],[left_front.x,left_front.y,left_back.x,left_back.y])
		
		front_point,back_point = carla.Location(front_point[0],front_point[1],0.0),carla.Location(back_point[0],back_point[1],0.0)
		right_point,left_point = carla.Location(right_point[0],right_point[1],0.0),carla.Location(left_point[0],left_point[1],0.0)
		
		dis2front,dis2back,dis2right,dis2left = ped_p.distance(front_point),ped_p.distance(back_point),ped_p.distance(right_point),ped_p.distance(left_point)

	return front_point,back_point,right_point,left_point,dis2front,dis2back,dis2right,dis2left

def go_forward(ped_ev,front_vec,right_vec,next_target,ped_p,i,dis2right,dis2left):
	
	Nh = ped_ev.graph.layers_height
	
	_region_node_dict = ped_ev.graph._region_node_dict
	node = _region_node_dict[next_target]
	Nw = 0
	for dic_layer in ped_ev.graph.node_gap:
		if node in dic_layer:
			Nw = dic_layer[node]
			break 
			
	next_region_right_mid_point,next_region_left_mid_point,(rf,rr,lf,lr) = get_Node_region_Left_Right_mid(right_vec,front_vec,Nh,Nw,next_target)	
	right_min_dis = dis2right if dis2right != -1 else sys.maxsize
	left_min_dis = dis2left if dis2left != -1 else sys.maxsize
	next_node_mid_point = get_mid_point(next_target)
	nxt_t_side = 'right' if vectorIsrightOf(next_node_mid_point,ped_p,right_vec) else 'left'
	
	_,ped_front_proj_point = cross_point([ped_p.x,ped_p.y,(ped_p+front_vec).x,(ped_p+front_vec).y],[next_region_right_mid_point.x,next_region_right_mid_point.y,next_region_left_mid_point.x,next_region_left_mid_point.y])
	ped_front_proj_point = carla.Location(ped_front_proj_point[0],ped_front_proj_point[1],0.0)
	
	if nxt_t_side == 'right': #行人若靠下個目標點左
		if ped_front_proj_point.distance(next_region_left_mid_point) <= ped_ev.close_move_around_vehicle[i]:   #若是與左邊點的邊界太近
			next_point = next_region_left_mid_point + ped_ev.close_move_around_vehicle[i]*right_vec if right_min_dis > ped_ev.close_move_around_vehicle[i] else next_region_left_mid_point + (right_min_dis)*right_vec
		else:
			next_point = ped_front_proj_point
	else:					  #行人若靠下個目標點右
		if ped_front_proj_point.distance(next_region_right_mid_point) <= ped_ev.close_move_around_vehicle[i]:  #若是與右邊點的邊界太近
			next_point = next_region_right_mid_point - ped_ev.close_move_around_vehicle[i]*right_vec if left_min_dis > ped_ev.close_move_around_vehicle[i] else next_region_right_mid_point - (left_min_dis)*right_vec
		else:
			next_point = ped_front_proj_point

	return next_point

def move_forward(ped_ev,front_vec,right_vec,ped_p):
	return ped_p + front_vec

def move_around(ped_ev,front_vec,right_vec,ped_p,next_target,i):
	fv = right_vec if vectorIsrightOf(get_mid_point(next_target),ped_p,right_vec) else negative_vector(right_vec)
	return ped_p + 3*right_vec if vectorIsrightOf(get_mid_point(next_target),ped_p,right_vec) else ped_p - 3*right_vec		

def go_to_next_target(ped_ev,current_region,target_region,current_front_node,ped_p,i,inLast,front_vec,right_vec,goal_sidewalk_region):
	
	front_point,back_point,right_point,left_point,dis2front,dis2back,dis2right,dis2left = getfourpoint(ped_ev,ped_p,current_region,front_vec,right_vec)
	moving_around = False

	if target_region != None and target_region != goal_sidewalk_region:
		if current_front_node == target_region:	
			next_point = go_forward(ped_ev,front_vec,right_vec,target_region,ped_p,i,dis2right,dis2left)#move_forward(ped_ev,front_vec,right_vec,ped_p)
		else:
			moving_around = True
			next_point = move_around(ped_ev,front_vec,right_vec,ped_p,target_region,i)
	else:
		if not inLast and target_region != goal_sidewalk_region: #死路
			ped_ev.stop[i] = True
			next_point = ped_p - front_vec
		else:
			next_point = ped_ev.ped_goal[i]
	
	#ped_ev.world.draw_point(next_point,1/29,carla.Color(255,0,0))
	
	return next_point,moving_around

def close_boundary(ped_ev,front_point,back_point,right_point,left_point,dis2front,dis2back,dis2right,dis2left,moving_around,unsafe,i,front_vec,right_vec):
	
	if (moving_around or unsafe):
		if dis2front != -1:
			if dis2front <= 0.3:
				ped_ev.block[i] = 1


	if dis2right != -1:
		if dis2right <= 0.5:
			ped_ev.right_block[i] = 1

			
	if dis2left != -1:
		if dis2left <= 0.5:
			ped_ev.left_block[i] = 1

def set_vehicle_force_point(ped_ev,current_region,i,front_point,back_point,right_point,left_point,moving_around,unsafe):

	front_vec,right_vec = ped_ev.front_v[i],ped_ev.right_v[i]
	
	if current_region != None:
		if moving_around or unsafe:
			ped_ev.gate_close[i] = ped_ev.pos()[i] - np.array([front_point.x,front_point.y])
		ped_ev.left_boundry[i] = ped_ev.pos()[i] - np.array([left_point.x,left_point.y])
		ped_ev.right_boundry[i] = ped_ev.pos()[i] - np.array([right_point.x,right_point.y])
		ped_ev.rear_boundry[i] = ped_ev.pos()[i] - np.array([back_point.x,back_point.y])
						
def collision_avoidance(ped_ev,inLast,inGraph,moving_around,unsafe,i,current_region,ped_p,front_vec,right_vec):

	front_point,back_point,right_point,left_point,dis2front,dis2back,dis2right,dis2left = getfourpoint(ped_ev,ped_p,current_region,front_vec,right_vec)
	close_boundary(ped_ev,front_point,back_point,right_point,left_point,dis2front,dis2back,dis2right,dis2left,moving_around,unsafe,i,front_vec,right_vec) 
	set_vehicle_force_point(ped_ev,current_region,i,front_point,back_point,right_point,left_point,moving_around,unsafe)