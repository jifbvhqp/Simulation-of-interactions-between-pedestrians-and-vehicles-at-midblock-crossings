import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
from tooluse import get_vec_proj_len,get_proj_len2,isCollision,line_in_rect,get_Node_Width_Height,CellCollision,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
import matplotlib.pyplot as plt 
from PIL import Image
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from numpy import random
from spawn_veh import spawn_car,spawn_motor,pre_spawn_motor,pre_spawn_car
from vehs_module import check_gap,get_vehs_info,Longitudinal_acc,sort_layer_arrays,sort_layer_array,sort_layer_arc_scene_arrays
from SAT import separating_axis_theorem
def pred_loc(ds,yaw,p_i,f_i,t,fixed_delta_t,sign,yaw_rate):
	pred_v = ds
	pred_yaw = yaw
	loc_vec = p_i
	f_vec = f_i
	traj = []
	traj.append(loc_vec)
	for frame in range(t):
		pred_acc = 2 * (1 - (pred_v/ds)**4)
		pred_v = pred_v + (fixed_delta_t * pred_acc)
		loc_vec = loc_vec + (fixed_delta_t * pred_v * f_vec)
		
		pred_yaw = pred_yaw + sign * yaw_rate * fixed_delta_t
		f_vec = carla.Vector3D(math.cos(math.radians(pred_yaw))*math.cos(0),math.sin(math.radians(pred_yaw))*math.cos(0),math.sin(0))
		traj.append(loc_vec)
	return loc_vec,traj 
def lane_point_gen_rect(p,lane_width_v,lane_front_v,rect_width,rect_height):
	p1 = p + lane_width_v*(rect_width/2) + lane_front_v*(rect_height/2)
	p2 = p + lane_width_v*(rect_width/2) - lane_front_v*(rect_height/2)
	p3 = p - lane_width_v*(rect_width/2) - lane_front_v*(rect_height/2)
	p4 = p - lane_width_v*(rect_width/2) + lane_front_v*(rect_height/2) 
	return (p1,p2,p3,p4)	
def get_waypoint_forward_right_width(carla_map,loc):
	waypoint = carla_map.get_waypoint(loc)
	fv = waypoint.transform.get_forward_vector()
	rv = waypoint.transform.get_right_vector()
	lane_width = waypoint.lane_width
	return fv,rv,lane_width,waypoint.transform.location 
def get_way_rect(carla_map,loc,rect_height):
	f,r,lane_width,w_loc = get_waypoint_forward_right_width(carla_map,loc)
	rect = lane_point_gen_rect(w_loc,r,f,lane_width,rect_height)   
	return rect

class veh_env:
	def __init__(self,world,save_folder,veh_arrive_rate,total_frame,updateL,spawn_end_points,car_spawn_points,motor_spawn_points,graph_attr,motor_pre_spawn,car_pre_spawn):				
		#World data
		self.world = world
		self.updateL = updateL
		self.spawn_end_points = spawn_end_points
		self.car_spawn_points = car_spawn_points
		self.motor_spawn_points = motor_spawn_points
		self.motor_pre_spawn = motor_pre_spawn
		self.car_pre_spawn = car_pre_spawn
		self.map = self.world.carla_world_map
		self.delta_time = self.world.fixed_delta_t
		self.save_folder = save_folder
		
		#Actors (agents)
		self.pedestrians = []
		self.vehicles = []
		self.ped_env = None
		self.graph = None
		
		#veh_arrival_rate
		self.veh_arrive_rate = veh_arrive_rate
		
		#For data collection
		self.total_frame = total_frame
		self.veh_data = []		 
		self.veh_rear = []
		self.veh_raduis = []	 
		self.bprint = []		
		self.frame_count = 0
		
		#Speed(m/s) , yaw_rate , Location , yaw angle(degrees) , acc
		self.velo_i = []	
		self.yaw_rate = []
		self.loc_i = []
		self.yaw_i = []
		self.a_idm = [] 
			
		#IDM parameter	
		self.para_random = [(5,6),(3,4),(3,4),(3,4)]	
		self.min_gap = []			 
		self.comb = []
		self.maxa = []
		self.desireS = []		
		self.tmp_desireS = [i for i in self.desireS]
		
		self.lanes_array = [[] for i in range(graph_attr[3])]
		self.lanes_array_for_get_leader = [[] for i in range(graph_attr[3])]
		#print(id(self.lanes_array),id(self.lanes_array_for_get_leader))
		self.veh_count = 0
		
		#For lane changing scenario_e		
		self.lanechanging = []
		
		#blocking vehicle index
		self.blocking_veh_idx = []
		self.target_yaw = []
		self.lane_change_phase = []
		self.lanechange_info = {}
		self.sign = []
		self.return_proj_vec = []
		self.driver_lane_change_info = []
		self.lanechange_side = []
		self.have_changed = []
		self.blocked_idx = []
		#command 
		self.lane_target_point = [carla.Location(100.441127,0,0.0),carla.Location(41.884701,-66.75,0.0)]
		self.veh_dir = []
		self.veh_command = []		
	def set_ped_env(self,ped_env):
		self.ped_env = ped_env
	def set_graph(self,graph):
		self.graph = graph	
	def get_Leader(self,i):
	
		leader_indeces = []
		leader_dis = []
		for lane_array_for_get_leader in self.lanes_array_for_get_leader:
			if i in lane_array_for_get_leader:
				current_idx = lane_array_for_get_leader.index(i)
				if current_idx > 0:
					if isinstance(lane_array_for_get_leader[current_idx - 1],int):
						if self.vehicles[lane_array_for_get_leader[current_idx - 1]].is_alive:
							leader_indeces.append(lane_array_for_get_leader[current_idx - 1])
					else:
						if self.ped_env.pedestrians[lane_array_for_get_leader[current_idx - 1][1]].is_alive:
							leader_indeces.append(lane_array_for_get_leader[current_idx - 1])
		
		
		w_p = self.world.carla_world_map.get_waypoint(self.vehicles[i].get_location())
		vehi_r = w_p.transform.get_right_vector()
		lanechange_idx = leader_indeces.copy()
		
		for idx in leader_indeces:
			if isinstance(idx,int):
				Leader_p = self.vehicles[idx].get_location() - self.vehicles[idx].get_transform().get_forward_vector() * self.veh_rear[idx]
				Leader_distance = carla.Location(self.loc_i[i].x,self.loc_i[i].y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))
			else:  
				Leader_p = self.ped_env.pedestrians[idx[1]].get_location()
				Leader_distance = carla.Location(self.loc_i[i].x,self.loc_i[i].y,0.0).distance(carla.Location(Leader_p.x,Leader_p.y,0.0))
			
			if self.lanechanging[i]:
				if self.lanechange_side[i] == 'right':
					if not vectorIsrightOf(Leader_p,self.vehicles[i].get_location(),vehi_r):
						lanechange_idx.remove(idx)
						continue
				else:
					if vectorIsrightOf(Leader_p,self.vehicles[i].get_location(),vehi_r):
						lanechange_idx.remove(idx)
						continue
						
			leader_dis.append(Leader_distance)
		
		if not self.lanechanging[i]:
			leader_idx = -1 if len(leader_indeces) == 0 else leader_indeces[leader_dis.index(min(leader_dis))]
		else:
			leader_idx = -1 if len(lanechange_idx) == 0 else lanechange_idx[leader_dis.index(min(leader_dis))]
		
		return leader_idx,lanechange_idx
	def save_data(self,speed_i,Leader_type,i):	  
		self.a_idm[i] = (speed_i - self.velo_i[i])/self.delta_time
		self.veh_data.append([self.frame_count,i,self.loc_i[i].x,self.loc_i[i].y,self.veh_raduis[i],self.veh_rear[i],self.velo_i[i],self.a_idm[i],Leader_type,self.yaw_i[i],self.bprint[i].id])		
	def draw_node(self,n,color = carla.Color(200,2,200)):
		p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
		p1.z,p2.z,p3.z,p4.z = 0.2,0.2,0.2,0.2
		self.world.draw_line(p1,p3,(1/29),color)
		self.world.draw_line(p3,p4,(1/29),color)
		self.world.draw_line(p4,p2,(1/29),color)
		self.world.draw_line(p2,p1,(1/29),color)
	def check_layer_state(self,i,rt,lt,rd,ld,p_i):
		for layer_count,layer_rect in enumerate(self.graph.layers_rect):	
		
			a_vertices = [(layer_rect[0].x,layer_rect[0].y),(layer_rect[1].x,layer_rect[1].y),(layer_rect[2].x,layer_rect[2].y),(layer_rect[3].x,layer_rect[3].y)]
			b_vertices = [(rt.x,rt.y),(lt.x,lt.y),(rd.x,rd.y),(ld.x,ld.y)]
			
			if self.have_changed[i] and layer_count == 7:
				if i in self.lanes_array_for_get_leader[7]:
					self.lanes_array_for_get_leader[7].remove(i)
				if i in self.lanes_array[7]:
					self.lanes_array[7].remove(i)
				continue
				
			if self.have_changed[i] and layer_count == 1:
				if i in self.lanes_array_for_get_leader[1]:
					self.lanes_array_for_get_leader[1].remove(i)
				if i in self.lanes_array[1]:
					self.lanes_array[1].remove(i)
				continue
			
			if separating_axis_theorem(a_vertices, b_vertices): 
				if not i in self.lanes_array[layer_count]:
					self.lanes_array[layer_count].append(i)
			
				if not i in self.lanes_array_for_get_leader[layer_count]:
					self.lanes_array_for_get_leader[layer_count].append(i)
			else:	   
				if i in self.lanes_array[layer_count]:
					self.lanes_array[layer_count].remove(i)
				
				if i in self.lanes_array_for_get_leader[layer_count]:
					self.lanes_array_for_get_leader[layer_count].remove(i)						
	def step(self):				  
		if self.save_folder == 'scenario_e':
			for i in range(0,len(self.vehicles)):
				if not self.vehicles[i].is_alive:
					continue	

				speed_i,f_i,r_i,rear_i,raduis_i,pi_f,pa,pc,pb,pd = get_vehs_info(self,i,self.vehicles[i].get_location())
				self.check_layer_state(i,pa,pb,pc,pd,self.vehicles[i].get_location())
	
		if self.save_folder == 'scenario_g':
			sort_layer_arc_scene_arrays(self)
		else:
			sort_layer_arrays(self) 
		
		for i in range(0,len(self.vehicles)):
			if not self.vehicles[i].is_alive:
				continue	 
			
			skip = 0
			for rect in self.spawn_end_points:						
				if isInRect(rect[0],rect[1],rect[2],rect[3],self.vehicles[i].get_location()):
					self.vehicles[i].destroy()
					
					#if self.save_folder == 'scenario_b':
					#	 inWhichLayer = []
					
					for layer_count,(lane_array,lane_array_for_get_leader) in enumerate(zip(self.lanes_array,self.lanes_array_for_get_leader)):
						if i in lane_array:
							lane_array.remove(i)
							
							#if self.save_folder == 'scenario_b':
							#	 inWhichLayer.append(layer_count)
							   
						if i in lane_array_for_get_leader:
							lane_array_for_get_leader.remove(i)
							
					skip = 1
					break
			
			
			if skip == 1:
				continue
	  
			veh_i = self.vehicles[i]		   
			p_i = veh_i.get_location()
			speed_i,f_i,r_i,rear_i,raduis_i,pi_f,pa,pb,pc,pd = get_vehs_info(self,i,p_i)
			
			Leader_index,leader_indeces = self.get_Leader(i)
			blocked = False
			
			blocking_vidx = -1
			blocking_dis = []
			
			for idx in leader_indeces:
				if idx in self.blocking_veh_idx and not i in self.blocking_veh_idx and self.vehicles[idx].get_location().distance(self.vehicles[i].get_location()) <= 25:
					blocked = True 
					blocking_vidx = idx					
					#blocking_dis.append(self.vehicles[idx].get_location().distance(self.vehicles[i].get_location()))

			acc_interac,desired_v,Leader_type,Leader_distance = Longitudinal_acc(self,Leader_index,i,p_i,f_i,r_i,pi_f,rear_i,raduis_i,speed_i,-1)
			acc = self.maxa[i] * (1 - (speed_i/self.desireS[i])**4) + acc_interac
			speed_i = speed_i + (self.delta_time * acc) 
			
			if not i in self.blocking_veh_idx:
				loc = self.loc_i[i] + (self.delta_time * speed_i * f_i)
			else:
				loc = self.loc_i[i]
				self.velo_i[i] = 0.0
				
			w_p = self.world.carla_world_map.get_waypoint(loc)
			if self.save_folder != 'scenario_e':
				w_pf = w_p.transform.rotation #target_yaw
				target_yaw = w_pf.yaw
				self.yaw_rate[i] = (target_yaw - self.yaw_i[i])/self.delta_time 
			else:			
				if not self.lanechanging[i]:
					gap_safe = True
					self.yaw_rate[i] = 0
					
					if blocked and Leader_distance <= 25:
						
						self.world.draw_point(carla.Location(p_i.x,p_i.y,3.0),1/29,carla.Color(255,0,0))
						self.driver_lane_change_info[i] = self.lanechange_info[blocking_vidx]
						check_safe_gap_layers_idxs = self.driver_lane_change_info[i][0]
						
						for idx in check_safe_gap_layers_idxs:
							Follower_gap,Leader_gap = check_gap(self,i,idx)
							
							if Follower_gap < 4 or Leader_gap < 4:
								gap_safe = False
								break

						if gap_safe:	
							self.lanechanging[i] = True
							self.lane_change_phase[i] = 1
							self.driver_lane_change_info[i] = self.lanechange_info[blocking_vidx]
							self.yaw_rate[i] = self.driver_lane_change_info[i][1]
							self.sign[i] = self.driver_lane_change_info[i][2]
							self.target_yaw[i] = self.driver_lane_change_info[i][3]
							self.return_proj_vec[i] = self.driver_lane_change_info[i][5]
							self.lanechange_side[i] = self.driver_lane_change_info[i][7]
							self.blocked_idx[i] = blocking_vidx
							
				if self.lanechanging[i]:
					va = p_i - self.return_proj_vec[i]
					vb = self.graph.graph_vec
					to_return_dir_dis = get_vec_proj_len(va,vb)
					
					if to_return_dir_dis > 0.05 and self.lane_change_phase[i] == 1:
						self.yaw_rate[i] = self.driver_lane_change_info[i][1] if speed_i >= 1.0 else 0.0
						if abs(self.target_yaw[i] - self.yaw_i[i]) <= 0.5:
							self.yaw_rate[i] = 0.0
					
					if to_return_dir_dis <= 0.05 and self.lane_change_phase[i] == 1:				
						self.target_yaw[i],self.sign[i] = self.driver_lane_change_info[i][4],-self.sign[i]
						self.lane_change_phase[i] = 2
					
					if abs(self.target_yaw[i]-self.yaw_i[i]) > 0.5 and self.lane_change_phase[i] == 2:				
						
						theta = abs(self.target_yaw[i]-self.yaw_i[i])
						ts = [100,90,80,70,60,50,40,30,20,10]
						pred_locs,y_error,omegas = [],[],[]
						
						for t in ts:				   
							omega = theta / (self.sign[i] * self.delta_time * t)					
							omega *= self.sign[i]
							omegas.append(omega)
							predloc,traj = pred_loc(speed_i,self.yaw_i[i],p_i,f_i,t,self.delta_time,self.sign[i],omega)
							pred_locs.append(predloc)
							
							va = predloc - p_i
							vb = self.graph.graph_vec
							proj_error = get_vec_proj_len(va,vb)

							y_error.append(proj_error)
						self.yaw_rate[i] = omegas[y_error.index(min(y_error))] if speed_i >= 1.0 else 0.0
						self.lane_change_phase[i] = 2
					
					
					if abs(self.target_yaw[i]-self.yaw_i[i]) <= 0.5 and self.lane_change_phase[i] == 2 and not blocked: 
						if self.blocked_idx[i] == 0 or self.blocked_idx[i] == 4:
							self.have_changed[i] = True
						self.lane_change_phase[i] = 0 
						self.lanechanging[i] = False
						self.yaw_rate[i] = 0.0					  
					
			
			yaw = self.yaw_i[i] + self.sign[i] * self.yaw_rate[i] * self.delta_time
			rot = carla.Rotation(0,yaw,0)	
			
			self.save_data(speed_i,Leader_type,i)
			#self.veh_command.append(carla.command.ApplyTransform(veh_i.id,carla.Transform(loc,rot)))
			veh_i.set_transform(carla.Transform(loc,rot))
			self.velo_i[i] = speed_i 
			self.loc_i[i] = loc				
			self.yaw_i[i] = yaw
			#self.alive_time[i] += 1
						
		self.frame_count += 1
		if self.frame_count % self.veh_arrive_rate == 0:
			choose_veh = ['auto','motor']
			chos_type = np.random.choice(choose_veh,1,p = [0.5,0.5])
			spawn_car(self) if chos_type == 'auto' else spawn_motor(self)

		for item in self.motor_pre_spawn:
			if item[3] == self.frame_count:
				pre_spawn_motor(self,item[0],item[1],item[2])	
				
		for item in self.car_pre_spawn:
			if item[4] == self.frame_count:
				pre_spawn_car(self,item[0],item[1],item[2],item[3]) 