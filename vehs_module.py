import glob
import os
import sys
import time
import numpy as np
import math
import random
from tooluse import get_vec_proj_len,get_proj_len2,get_Node_Width_Height,CellCollision,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
import matplotlib.pyplot as plt 
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from numpy import random
def sort_layer_arc_scene_arrays(veh_ev):
	
	sorted_arrays_for_get_leader = []
	sorted_arrays_for_graph = []
	
	for layer_count,(lanes_array_for_get_leader,layer_mid_point) in enumerate(zip(veh_ev.lanes_array_for_get_leader,veh_ev.graph.layer_mid_point)):

		sorted_array_for_get_leader = []
		dis_to_end_for_get_leader = []
		agent_idx_for_get_leader = []
		
		sorted_array_for_graph = []
		dis_to_end_for_graph = []
		agent_idx_for_graph = []
		
		for idx in lanes_array_for_get_leader:
			if isinstance(idx,int):
				vp = veh_ev.vehicles[idx].get_location() + (veh_ev.veh_rear[idx] - 0.5) * veh_ev.vehicles[idx].get_transform().get_forward_vector()
				if veh_ev.veh_dir[idx] == 0:
					dis = veh_ev.lane_target_point[0].distance(carla.Location(vp.x,vp.y,0.0))
				else:					 
					dis = veh_ev.lane_target_point[1].distance(carla.Location(vp.x,vp.y,0.0))
					
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)
			
				dis_to_end_for_graph.append(dis)
				agent_idx_for_graph.append(idx)
			else:
				p = veh_ev.ped_env.pedestrians[idx[1]].get_location()
				
				if layer_count >=7:
					dis = veh_ev.lane_target_point[1].distance(carla.Location(p.x,p.y,0.0))
				else:
					dis = veh_ev.lane_target_point[0].distance(carla.Location(p.x,p.y,0.0))	
					
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)


		if len(dis_to_end_for_get_leader) > 0:
			for i in range(len(dis_to_end_for_get_leader)):
				sorted_array_for_get_leader.append(agent_idx_for_get_leader[np.argsort(dis_to_end_for_get_leader)[i]])
		sorted_arrays_for_get_leader.append(sorted_array_for_get_leader)
		
		
		if len(dis_to_end_for_graph) > 0:
			for i in range(len(dis_to_end_for_graph)):
				sorted_array_for_graph.append(agent_idx_for_graph[np.argsort(dis_to_end_for_graph)[i]])
		sorted_arrays_for_graph.append(sorted_array_for_graph)
	
	veh_ev.lanes_array = sorted_arrays_for_graph
	veh_ev.lanes_array_for_get_leader = sorted_arrays_for_get_leader
	
def sort_layer_array(veh_ev,copy_array,layer_count):
	
	sorted_array_for_get_leader = []
	dis_to_end_for_get_leader = []
	agent_idx_for_get_leader = []
	
	for idx in copy_array:
		if isinstance(idx,int):
			if dot(veh_ev.vehicles[idx].get_transform().get_forward_vector(),veh_ev.graph.graph_width_vec) > 0:	
				p1 = veh_ev.vehicles[idx].get_location() + veh_ev.veh_rear[idx] * veh_ev.graph.graph_width_vec - 0.5 * veh_ev.graph.graph_width_vec
				p3 = veh_ev.graph.layer_mid_point[layer_count][0]
				p4 = veh_ev.graph.layer_mid_point[layer_count][1]
				d = get_proj_len2(p1,p4,p3)
				proj_p = p3 + d * normalize(p4 - p3)
				dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))
				
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)
			else:
				p1 = veh_ev.vehicles[idx].get_location() - veh_ev.veh_rear[idx] * veh_ev.graph.graph_width_vec + 0.5 * veh_ev.graph.graph_width_vec
				p3 = veh_ev.graph.layer_mid_point[layer_count][0]
				p4 = veh_ev.graph.layer_mid_point[layer_count][1]
				d = get_proj_len2(p1,p3,p4)
				proj_p = p4 + d * normalize(p3 - p4)
				dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))
				
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)
		else:
			p1 = veh_ev.ped_env.pedestrians[idx[1]].get_location()
			p3 = veh_ev.graph.layer_mid_point[layer_count][0]
			p4 = veh_ev.graph.layer_mid_point[layer_count][1]
			d = get_proj_len2(p1,p4,p3)
			proj_p = p3 + d * normalize(p4 - p3)
			#veh_ev.world.draw_point(proj_p,(1/29),carla.Color(255,0,0))
			if dot(veh_ev.world.carla_world_map.get_waypoint(proj_p).transform.get_forward_vector(),veh_ev.graph.graph_width_vec) > 0:
				dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))
				#veh_ev.world.draw_line(proj_p,p4,(1/29),carla.Color(255,0,0))
				
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)
			else:
				dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))
				#veh_ev.world.draw_line(proj_p,p3,(1/29),carla.Color(255,0,0))
	
				dis_to_end_for_get_leader.append(dis)
				agent_idx_for_get_leader.append(idx)
				
	if len(dis_to_end_for_get_leader) > 0:
		for i in range(len(dis_to_end_for_get_leader)):
			sorted_array_for_get_leader.append(agent_idx_for_get_leader[np.argsort(dis_to_end_for_get_leader)[i]])			  
		
	return sorted_array_for_get_leader

def check_gap(veh_ev,i,layer_count):
	
	Leader_gap = veh_ev.graph.graph_width
	Follower_gap = veh_ev.graph.graph_width
	
	copy_array = veh_ev.lanes_array_for_get_leader[layer_count].copy()
	
	if not i in copy_array:
		copy_array.append(i)

	sorted_array = sort_layer_array(veh_ev,copy_array,layer_count)
	index = sorted_array.index(i)
	
	if index != len(sorted_array) - 1:	
		follower_idx = sorted_array[sorted_array.index(i)+1]
		
		if isinstance(follower_idx,int):
			pa = veh_ev.vehicles[i].get_location()-veh_ev.vehicles[i].get_transform().get_forward_vector()*veh_ev.veh_rear[i]
			pb = veh_ev.vehicles[follower_idx].get_location()+veh_ev.vehicles[follower_idx].get_transform().get_forward_vector()*veh_ev.veh_rear[follower_idx]
			va = pb - pa
			vb = veh_ev.graph.graph_width_vec
			Follower_gap = get_vec_proj_len(va,vb)
		else:
			pa = veh_ev.vehicles[i].get_location()-veh_ev.vehicles[i].get_transform().get_forward_vector()*veh_ev.veh_rear[i]
			pb = veh_ev.ped_env.pedestrians[follower_idx[1]].get_location()
			va = pb - pa
			vb = veh_ev.graph.graph_width_vec
			Follower_gap = get_vec_proj_len(va,vb)
		
		veh_ev.world.draw_line(pa,pb,1/29,carla.Color(0,255,0))
	#	print(sorted_array,index,index != len(sorted_array) - 1,follower_idx)
	#	
	#	if self.vehicles[follower_idx].is_alive:
	#		p_j = self.vehicles[follower_idx].get_location()
	#		p_j.z = 3.0
	#		self.world.draw_point(p_j,1/29,carla.Color(0,0,255))
	#		#
	#else:
	#	print(sorted_array,index,index != len(sorted_array) - 1)
	elif index != 0:
		leader_index = sorted_array[sorted_array.index(i)-1]
		
		if isinstance(leader_index,int):
			pa = veh_ev.vehicles[i].get_location()+veh_ev.vehicles[i].get_transform().get_forward_vector()*veh_ev.veh_rear[i]
			pb = veh_ev.vehicles[leader_index].get_location()-veh_ev.vehicles[leader_index].get_transform().get_forward_vector()*veh_ev.veh_rear[leader_index]
			va = pb - pa
			vb = veh_ev.graph.graph_width_vec
			Leader_gap = get_vec_proj_len(va,vb)
		else:
			pa = veh_ev.vehicles[i].get_location()-veh_ev.vehicles[i].get_transform().get_forward_vector()*veh_ev.veh_rear[i]
			pb = veh_ev.ped_env.pedestrians[leader_index[1]].get_location()
			va = pb - pa
			vb = veh_ev.graph.graph_width_vec
			Follower_gap = get_vec_proj_len(va,vb)
			
		veh_ev.world.draw_line(pa,pb,1/29,carla.Color(0,255,255))
		
	return Follower_gap,Leader_gap
	
def sort_layer_arrays(veh_ev):
	
	sorted_arrays_for_get_leader = []
	sorted_arrays_for_graph = []
	
	for layer_count,(lanes_array_for_get_leader,layer_mid_point) in enumerate(zip(veh_ev.lanes_array_for_get_leader,veh_ev.graph.layer_mid_point)):

		sorted_array_for_get_leader = []
		dis_to_end_for_get_leader = []
		agent_idx_for_get_leader = []
		
		sorted_array_for_graph = []
		dis_to_end_for_graph = []
		agent_idx_for_graph = []
		
		for idx in lanes_array_for_get_leader:
			if isinstance(idx,int):
				if dot(veh_ev.vehicles[idx].get_transform().get_forward_vector(),veh_ev.graph.graph_width_vec) > 0:	
				
					p1 = veh_ev.vehicles[idx].get_location() + veh_ev.veh_rear[idx] * veh_ev.graph.graph_width_vec - 0.5 * veh_ev.graph.graph_width_vec
					p3 = layer_mid_point[0]
					p4 = layer_mid_point[1]
					d = get_proj_len2(p1,p4,p3)
					proj_p = p3 + d * normalize(p4 - p3)
					dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))
					#veh_ev.world.draw_point(proj_p,(1/29),carla.Color(255,0,0))
					
					dis_to_end_for_get_leader.append(dis)
					agent_idx_for_get_leader.append(idx)
					
					dis_to_end_for_graph.append(dis)
					agent_idx_for_graph.append(idx)
				else:
					
					p1 = veh_ev.vehicles[idx].get_location() - veh_ev.veh_rear[idx] * veh_ev.graph.graph_width_vec	+ 0.5 * veh_ev.graph.graph_width_vec
					p3 = layer_mid_point[0]
					p4 = layer_mid_point[1]
					d = get_proj_len2(p1,p3,p4)
					proj_p = p4 + d * normalize(p3 - p4)
					dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))
					#veh_ev.world.draw_point(proj_p,(1/29),carla.Color(255,0,0))

					dis_to_end_for_get_leader.append(dis)
					agent_idx_for_get_leader.append(idx)
					
					dis_to_end_for_graph.append(dis)
					agent_idx_for_graph.append(idx)
			else:
				p1 = veh_ev.ped_env.pedestrians[idx[1]].get_location()
				p3 = layer_mid_point[0]
				p4 = layer_mid_point[1]
				d = get_proj_len2(p1,p4,p3)
				proj_p = p3 + d * normalize(p4 - p3)
				#veh_ev.world.draw_point(proj_p,(1/29),carla.Color(255,0,0))
				if dot(veh_ev.world.carla_world_map.get_waypoint(proj_p).transform.get_forward_vector(),veh_ev.graph.graph_width_vec) > 0:
					dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))
					#veh_ev.world.draw_line(proj_p,p4,(1/29),carla.Color(255,0,0))
					
					dis_to_end_for_get_leader.append(dis)
					agent_idx_for_get_leader.append(idx)
				else:
					dis = carla.Location(proj_p.x,proj_p.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))
					#veh_ev.world.draw_line(proj_p,p3,(1/29),carla.Color(255,0,0))
		
					dis_to_end_for_get_leader.append(dis)
					agent_idx_for_get_leader.append(idx)
			

		if len(dis_to_end_for_get_leader) > 0:
			for i in range(len(dis_to_end_for_get_leader)):
				sorted_array_for_get_leader.append(agent_idx_for_get_leader[np.argsort(dis_to_end_for_get_leader)[i]])
		sorted_arrays_for_get_leader.append(sorted_array_for_get_leader)
		
		
		if len(dis_to_end_for_graph) > 0:
			for i in range(len(dis_to_end_for_graph)):
				sorted_array_for_graph.append(agent_idx_for_graph[np.argsort(dis_to_end_for_graph)[i]])
		sorted_arrays_for_graph.append(sorted_array_for_graph)
	
	veh_ev.lanes_array = sorted_arrays_for_graph
	veh_ev.lanes_array_for_get_leader = sorted_arrays_for_get_leader

def get_vehs_info(veh_ev,i,p_i):

	speed_i = veh_ev.velo_i[i]
	f_i = veh_ev.vehicles[i].get_transform().get_forward_vector()
	r_i = veh_ev.vehicles[i].get_transform().get_right_vector()
	rear_i = veh_ev.veh_rear[i]
	raduis_i = veh_ev.veh_raduis[i]
	car_front_point = p_i + rear_i*f_i
	car_back_point = p_i - rear_i*f_i
	
	pi_f = car_front_point	 
	pa = car_front_point + r_i * raduis_i 
	pb = car_front_point - r_i * raduis_i 
	pc = car_back_point + r_i * raduis_i
	pd = car_back_point - r_i * raduis_i
	
	return speed_i,f_i,r_i,rear_i,raduis_i,pi_f,pa,pb,pc,pd

def Longitudinal_acc(veh_ev,Leader_index,i,p_i,f_i,r_i,pi_f,rear_i,raduis_i,speed_i,debug_index):

	acc_interac = 0.0
	Leader_distance = sys.maxsize
	Leader_type = 'veh'
	desired_v = veh_ev.desireS[i]
	
	if Leader_index != -1:
		if isinstance(Leader_index,int):

			Leader_p = veh_ev.vehicles[Leader_index].get_location() - veh_ev.vehicles[Leader_index].get_transform().get_forward_vector() * veh_ev.veh_rear[Leader_index]
			#if i == debug_index:
			veh_ev.world.draw_line(Leader_p,p_i,1/29,carla.Color(255,255,0))
			speed_j = veh_ev.velo_i[Leader_index]
			Leader_type = 'veh'
			gap = veh_ev.min_gap[i]
			T = 1.0			  
			va = Leader_p - carla.Location(pi_f.x,pi_f.y,0.0)
			vb = veh_ev.graph.graph_width_vec
			Leader_distance = get_vec_proj_len(va,vb)
			
			if veh_ev.save_folder == 'scenario_g':
				Leader_distance = carla.Location(pi_f.x,pi_f.y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))
			
			#carla.Location(pi_f.x,pi_f.y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))
			acc_interac = -veh_ev.maxa[i]*(max((gap + speed_i*T + ((speed_i*(speed_i-speed_j))/(2*(veh_ev.maxa[i]*veh_ev.comb[i])**0.5))),0)/max(Leader_distance,gap)) ** 2
			
		else:
		
			Leader_p = veh_ev.ped_env.pedestrians[Leader_index[1]].get_location()
			#if i == debug_index:
			#veh_ev.world.draw_line(Leader_p,p_i,1/29,carla.Color(0,255,0))
			speed_j = 0
			Leader_type = 'ped'
			gap = veh_ev.min_gap[i]
			T = 1.0		
			va = Leader_p - carla.Location(pi_f.x,pi_f.y,0.0)
			vb = veh_ev.graph.graph_width_vec			 
			Leader_distance = get_vec_proj_len(va,vb)#carla.Location(pi_f.x,pi_f.y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))			
			
			if veh_ev.save_folder == 'scenario_g':
				Leader_distance = carla.Location(pi_f.x,pi_f.y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))			 
			
			acc_interac = -veh_ev.maxa[i]*(max((gap + speed_i*T + ((speed_i*(speed_i-speed_j))/(2*(veh_ev.maxa[i]*veh_ev.comb[i])**0.5))),0)/max(Leader_distance,gap)) ** 2
			#Leader_p = veh_ev.pedestrians[Leader_index[1]].get_location()
			#Leader_distance = abs(Leader_p.x - pi_f.x)
			##veh_ev.world.draw_line(Leader_p,p_i,0.2,carla.Color(255,255,0))	 
			#Leader_type = 'ped'
			#if Leader_distance <= 10:
			#	 
			#	 min_ped_gap = 3.0
			#	 L = 1.0
			#	 t0 = abs(Leader_p.y - (p_i + r_i*raduis_i).y)/veh_ev.ped_ev.max_speed_multiplier[Leader_index[1]]	   
			#	 Leader_p = veh_ev.pedestrians[Leader_index[1]].get_location()
			#	 d = abs(Leader_p.x - pi_f.x) - 1.0 if abs(Leader_p.x - pi_f.x) > min_ped_gap else min_ped_gap
			#
			#	 desired_v = (d-speed_i/L+speed_i/L*math.exp(-L*t0))/max((t0-1/L+1/L*math.exp(-L*t0)),0.1)
			#	 desired_v = min(max(desired_v,0),veh_ev.desireS[i])
			#	 desired_v *= 0.3
			#	 desired_v = 0 if d <= min_ped_gap else desired_v
				
			
	return acc_interac,desired_v,Leader_type,Leader_distance