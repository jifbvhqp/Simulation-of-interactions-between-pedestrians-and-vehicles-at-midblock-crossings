import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
import matplotlib.pyplot as plt 
from tooluse import nearest_next_nodes,get_vec_proj_len,vecs2angle,get_proj_len2,draw_circles,get_Node_region_Left_Right_mid,regions_distance,get_front_region,get_len,get_Node_Width_Height,NormalizeData,negative_vector,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
import csv
from numpy import random
from force import DrivingForce,SocialForce,SocialForce2,gateForce,Left_region_boundry_force,Right_region_boundry_force,Rear_region_boundry_force
from Compute_Edge_Weight import compute_edge_weight
from spawn_ped import spawn_pedestrian,spawn_pedestrian2
from peds_module import getfourpoint,init,check_state,Caculate_and_Get_target_Region,go_to_next_target,collision_avoidance
class ped_env:
	def __init__(self,world,save_folder,ped_arrive_rate,total_frame,updateL,graph_attr,ped_spawn_point = [],ped_side_walk = [],ped_direction = [],ped_spawn_end_points = []):
		#World data
		self.world = world
		self.map = self.world.carla_world_map		
		self.ped_spawn_point = ped_spawn_point
		self.ped_side_walk = ped_side_walk
		self.ped_direction = ped_direction
		self.ped_spawn_end_points = ped_spawn_end_points
		self.delta_time = self.world.fixed_delta_t
		self.updateL = updateL
		self.graph = self.updateL.graph
		self.save_folder = save_folder
		#Actors (agents)
		self.pedestrians = []
		self.vehicles = []
		self.veh_env = updateL.veh_ev
		
		#ped_arrival_rate
		self.ped_arrive_rate = ped_arrive_rate
		
		#For data collection
		self.total_frame = total_frame		
		self.frame_count = 0
		self.ped_data = []
		
		#social force parameter		
		self.goal_threshold = 0.2
		self.tau = 0.5
		self.pedestrain_r = 0.3
		
		#speed range
		self.speed_under = 1.1
		self.speed_up = 2.0		  
		self.initial_speeds = None
		self.max_speeds = None	
		self.max_speed_multiplier = []
		
		#state
		self.state = None
		
		#pedestrian front , right vector
		self.front_v = []
		self.right_v = []
		
		#pedestrian safe distance with vehicle
		self.close_move_around_vehicle = []
		
		#pedestrian goal		
		self.ped_goal = []
		
		#pedestrian type
		self.ped_type = []
		
		#pedestrian stop
		self.stop = []
		
		#pedestrian temp goal
		self.temp_goal = []
		self.temp_type = []
		
		self.node_front_force = np.zeros((self.size(),2))
		self.node_left_force = np.zeros((self.size(),2))
		self.node_right_force = np.zeros((self.size(),2))
		self.node_corner_force = np.zeros((self.size(),2))
		self.gate_close = np.zeros((self.size(),2))

		self.front_block_point = np.zeros((self.size(),2))
		self.left_block_point = np.zeros((self.size(),2))
		self.right_block_point = np.zeros((self.size(),2))
		self.rear_block_point = np.zeros((self.size(),2))

		self.block = np.zeros((self.size()))
		self.left_block = np.zeros((self.size()))
		self.right_block = np.zeros((self.size()))
		self.rear_block = np.zeros((self.size()))
		
		self.ped_front_block = np.zeros((self.size()))
		self.ped_front_block_point = np.zeros((self.size(),2))
		self.blockage_i = np.zeros((self.size()))
		
		self.left_boundry = np.zeros((self.size(),2))
		self.right_boundry = np.zeros((self.size(),2))
		self.rear_boundry = np.zeros((self.size(),2))	
		
		self.w1 = 0.0
		self.w2 = 0.0
		self.w3 = 0.0
		self.w4 = 0.0
		self.w5 = 0.0
		
		self.ped_type1_in_region = {}
		self.ped_type2_in_region = {}
		self.player_ped_idxs = [] 
		self.in_layer_id = []
		
		#command 
		self.peds_command = []
	
	def arc(self):	
		for i in range(0,self.size()):	
			if not self.pedestrians[i].is_alive:
				continue			  
				
			cpa = carla.Location(65.27294921875,-54.009620666503906,1.0)
			cpb = carla.Location(96.32307434082031,-23.087177276611328,1.0)
			
			self.world.draw_point(cpa,1/29,carla.Color(255,0,255))	
			self.world.draw_point(cpb,1/29,carla.Color(0,0,255))	
			
			fv = carla.Vector3D(1,0,0)
			rv = carla.Vector3D(0,1,0)
			
			_,ccp = cross_point([cpa.x,cpa.y,cpa.x+rv.x,cpa.y+rv.y],[cpb.x,cpb.y,cpb.x+fv.x,cpb.y+fv.y])
			ccp = carla.Location(ccp[0],ccp[1],1.0)

			r = cpb.distance(ccp) - 1.25/2
			
			Rs = [r + 1.25 * i for i in range(0,16)]
			arc_points_set = [[] for i in range(0,16)]
			
			mid = carla.Location(ccp.x + r * math.cos(math.radians(315)),ccp.y + r * math.sin(math.radians(315)),1.0)
			vmid = normalize(mid - ccp)
			
			pvec = self.pedestrians[i].get_location() - ccp
			vlen = math.sqrt(pvec.x**2 + pvec.y**2)			
						
			arc_graph_mid_p_1 = []
			arc_graph_mid_p_2 = []
			
			for cnt,pset in enumerate(arc_points_set):
				if cnt < len(arc_points_set) - 1:
					if vlen < Rs[cnt+1] and vlen > Rs[cnt] and vecs2angle(pvec,vmid) <= 45:
						self.in_layer_id[i] = cnt

	def set_para(self,w1,w2,w3,w4,w5):
		self.w1 = w1
		self.w2 = w2
		self.w3 = w3
		self.w4 = w4
		self.w5 = w5
	def draw_node(self,n,color = carla.Color(0,0,0)):
		p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
		p1.z,p2.z,p3.z,p4.z = 0.2,0.2,0.2,0.2
		self.world.draw_line(p1,p2,1/29,color)
		self.world.draw_line(p2,p3,1/29,color)
		self.world.draw_line(p3,p4,1/29,color)
		self.world.draw_line(p4,p1,1/29,color)
	def size(self):
		return len(self.pedestrians)		
	def update(self,state):
		self.state = state	
	def pos(self) -> np.ndarray:
		return self.state[:, 0:2]
	def vel(self) -> np.ndarray:
		return self.state[:, 2:4]
	def goal(self) -> np.ndarray:
		return self.state[:, 4:6]
	def speeds(self):
		return stateutils.speeds(self.state)
	def desired_directions(self):
		return stateutils.desired_directions(self.state)[0] 
	@staticmethod
	def capped_velocity(desired_velocity, max_velocity):
		"""Scale down a desired velocity to its capped speed."""
		desired_speeds = np.linalg.norm(desired_velocity, axis=-1)
		factor = np.minimum(1.0, max_velocity / desired_speeds)
		factor[desired_speeds == 0] = 0.0
		return desired_velocity * np.expand_dims(factor, -1)		
	def getForce(self):
		if self.size() > 0: 
			return DrivingForce(self) + 2.0*(SocialForce(self)+SocialForce2(self)) + 2.0*(gateForce(self) + Left_region_boundry_force(self) + Right_region_boundry_force(self) + Rear_region_boundry_force(self))
		else:
			return				
	def process_vel(self,desired_velocity):
	
		for i in range(self.size()):	   
			
			vel_vec = carla.Vector3D(desired_velocity[i][0],desired_velocity[i][1],0.0)
			if self.block[i]==1:
				proj_len = get_vec_proj_len(vel_vec,self.front_v[i])
				vel_vec = vel_vec - proj_len * self.front_v[i]
				desired_velocity[i][0] = vel_vec.x
				desired_velocity[i][1] = vel_vec.y
			
			#if self.right_block[i] == 1:	
			#	proj_len = get_vec_proj_len(vel_vec,self.right_v[i])
			#	
			#	if vectorIsrightOf(self.pedestrians[i].get_location()+vel_vec*self.delta_time,self.pedestrians[i].get_location(),self.right_v[i]):
			#		vel_vec = vel_vec + proj_len * self.right_v[i]
			#		desired_velocity[i][0] = vel_vec.x
			#		desired_velocity[i][1] = vel_vec.y
			#
			#if self.left_block[i] == 1:
			#	proj_len = get_vec_proj_len(vel_vec,self.right_v[i])
			#	
			#	if not vectorIsrightOf(self.pedestrians[i].get_location()+vel_vec*self.delta_time,self.pedestrians[i].get_location(),self.right_v[i]):
			#		vel_vec = vel_vec - proj_len * self.right_v[i]
			#		desired_velocity[i][0] = vel_vec.x
			#		desired_velocity[i][1] = vel_vec.y				
				
		return desired_velocity
	def move(self):	   
		for i in range(self.size()):				
			if not self.pedestrians[i].is_alive:
				continue
			
			if self.save_folder == 'scenario_g':
				if i in self.player_ped_idxs:
					current_pos = carla.Location(self.pedestrians[i].get_location().x,self.pedestrians[i].get_location().y,0)
					self.pos()[i][0] = current_pos.x
					self.pos()[i][1] = current_pos.y
					continue
			
			next_pos = carla.Location(self.pos()[i][0],self.pos()[i][1],1.0)
			if self.stop[i]:
				current_pos = carla.Location(self.pedestrians[i].get_location().x,self.pedestrians[i].get_location().y,0)
				self.pos()[i][0] = current_pos.x
				self.pos()[i][1] = current_pos.y	
				#self.pedestrians[i].set_transform(carla.Transform(next_pos,carla.Rotation(0,taryaw,0)))
				#self.peds_command.append(carla.command.ApplyWalkerControl(self.pedestrians[i].id,carla.WalkerControl(carla.Vector3D(0,0,0),0.0,False)))
				
			else:		
				current_pos = carla.Location(self.pedestrians[i].get_location().x,self.pedestrians[i].get_location().y,0)
				direction = normalize(next_pos - current_pos)# if self.ped_front_block[i] == 0 else self.pedestrians[i].get_transform().get_forward_vector()
				taryaw = math.degrees(math.atan2(direction.y, direction.x))
				self.pedestrians[i].set_transform(carla.Transform(next_pos,carla.Rotation(0,taryaw,0)))
				#self.peds_command.append(carla.command.ApplyTransform(self.pedestrians[i].id,carla.Transform(next_pos,carla.Rotation(0,taryaw,0))))	  
	def step(self):
		"""Move peds according to forces"""
		# desired velocity
		if self.size() > 0:
		
			if self.save_folder == 'scenario_g':
				self.arc()
				
			self.getNextGoal() 
			
			force = self.getForce()

			desired_velocity = self.vel() + self.delta_time*force
			desired_velocity = self.capped_velocity(desired_velocity,self.max_speeds)
			
			# stop when arrived
			desired_velocity[stateutils.desired_directions(self.state)[1] < 0.5] = [0,0]		
			desired_velocity = self.process_vel(desired_velocity)
			
			# update state
			next_state = self.state.copy()
			next_state[:, 0:2] += desired_velocity * self.delta_time	#update pos
			next_state[:, 2:4] = desired_velocity						#update velocity

			self.update(next_state)		
			self.move()		
			
		self.frame_count += 1					
		if self.frame_count % self.ped_arrive_rate == 0 and len(self.ped_spawn_point) > 0:
			#if len(self.pedestrians) < 1:
			spawn_pedestrian(self)		
			
		for item in self.ped_spawn_end_points:
			if item[2] == self.frame_count:
				spawn_pedestrian2(self,item[0],item[1],item[3],item[4])			
	def get_Next_target(self,next_nodes,degree_of_passages,travel_lengths,decayed_rate,Occupancy_rate,region_flow,dead_road,i,search_gap):
		total = []
		index = -1
		for idx,n in enumerate(next_nodes):
			w = self.w1*degree_of_passages[idx] + self.w2*travel_lengths[idx] + self.w3*decayed_rate[idx] + self.w4*region_flow[idx] + self.w5*Occupancy_rate[idx]
			
			#if dead_road[idx]:
			#	w = sys.float_info.min
			total.append(w)			
		index = np.argmax(total)		
		return next_nodes[np.argmax(total)],index,total		
	def init_ped(self,i):
		if self.pedestrians[i].get_location().distance(carla.Location(self.ped_goal[i].x,self.ped_goal[i].y,self.pedestrians[i].get_location().z)) < 0.5:
			if len(self.temp_goal[i]) > 1:
				
				self.temp_goal[i].pop(0)
				self.ped_goal[i] = self.temp_goal[i][0]
				self.ped_type[i] = self.temp_type[i][0]
				self.temp_type[i].pop(0)				   
				self.state[i, 4:6] = np.array([self.ped_goal[i].x,self.ped_goal[i].y])
				print(self.ped_goal[i],self.ped_type[i])
			else:
				self.ped_type[i] = self.temp_type[i][0]
				self.temp_type[i].pop(0)				 
				self.pos()[i] = [-9999,-9999]
				self.pedestrians[i].destroy()
			
		for v in self.ped_direction:
			if self.ped_type[i] == v[2]:
				self.front_v[i] = v[0]
				self.right_v[i] = v[1]
				break				
	def save_data(self,crossing_state,i):
		self.ped_data.append([self.frame_count,i,crossing_state,self.ped_type[i],self.pos()[i][0],self.pos()[i][1],self.vel()[i][0],self.vel()[i][1],self.w1,self.w2,self.w3,self.w4,self.w5])	
	def getNextGoal(self):
		
		
		G = self.graph.G
		GT = self.graph.GT
		init(self)
		self.ped_type1_in_region = {region:[] for region in G}
		self.ped_type2_in_region = {region:[] for region in G}
				
		for region in G:
			for i in range(0,self.size()):
				if self.ped_type[i] == 1:
					if isInRect(region[0],region[1],region[2],region[3],self.pedestrians[i].get_location()):
						self.ped_type1_in_region[region].append(i)
				elif self.ped_type[i] == 2:
					if isInRect(region[0],region[1],region[2],region[3],self.pedestrians[i].get_location()):
						self.ped_type2_in_region[region].append(i)
										
		for i in range(0,self.size()):	
		
			if not self.pedestrians[i].is_alive:
				continue

			self.init_ped(i)
			
			if not self.pedestrians[i].is_alive:
				continue

			self.stop[i] = False
			ped_p = carla.Location(self.pedestrians[i].get_location().x,self.pedestrians[i].get_location().y,0.0)			
 
			if self.save_folder == 'scenario_g':
				if self.ped_type[i] == 3:
					self.save_data(5,i)
					continue
				if self.ped_type[i] == 2 and self.in_layer_id[i] == -1:
					next_layer_id = 14
				elif self.ped_type[i] == 1 and self.in_layer_id[i] == -1:
					next_layer_id = 0
				elif self.in_layer_id[i] != -1:
					next_layer_id = self.in_layer_id[i] - 1 if self.ped_type[i] == 2 else self.in_layer_id[i] + 1
				
				for layer_count,lanes_array_for_get_leader in enumerate(self.veh_env.lanes_array_for_get_leader):
					if layer_count == self.in_layer_id[i] or layer_count == next_layer_id:		 
						if not ('ped',i) in lanes_array_for_get_leader:
							lanes_array_for_get_leader.append(('ped',i))
					else:
						if ('ped',i) in lanes_array_for_get_leader:
							lanes_array_for_get_leader.remove(('ped',i))
				self.save_data(5,i)
				continue
				
			crossing_state,goal_sidewalk_region,src_sidewalk_region,in_layer_id = check_state(self,ped_p,i)
			last_layer_regions = self.graph.layers_region[len(self.graph.layers_region)-1] if self.ped_type[i] == 1 else self.graph.layers_region[0]
			
			if self.ped_type[i] == 3:
				self.save_data(crossing_state,i)
				continue
			
			#self.max_speed_multiplier[i] = 2.25
			#self.max_speeds = self.max_speed_multiplier * np.ones((self.size()))
			
			next_regions = []
			current_region = None
			inLast = False
			inGraph = False			
			next_layer_id = -1
			

			if crossing_state == 0:				  
				current_region = None
				next_regions = self.graph.layers_region[0] if self.ped_type[i] == 1 else self.graph.layers_region[len(self.graph.layers_region)-1]
				inLast = False
				inGraph = False
				next_layer_id = 0 if self.ped_type[i] == 1 else self.graph.layers_num - 1
				
			elif crossing_state == 1:
				current_region = None
				
				#掃描全region判斷行人在哪個region內
				for layer in self.graph.layers_region:
					for region in layer:
						if isInRect(region[0],region[1],region[2],region[3],ped_p):
							current_region = region
							break 
							
				#判斷行人是否在最後一層內，若是的話，下層可到達目標region為目標人行道的region。若不是的話，下層可到達目標region為graph內下層有邊連接的region
				if not current_region in last_layer_regions:
					if self.ped_type[i] == 1:
						next_regions = G[current_region] if current_region in G else []
					else:
						next_regions = GT[current_region] if current_region in GT else []
				else:
					next_regions = [goal_sidewalk_region]
				
				#尚未到最後階段，正在動態圖內
				inLast = False
				inGraph = True
				
				if not current_region in last_layer_regions:
					next_layer_id = in_layer_id + 1 if self.ped_type[i] == 1 else in_layer_id - 1
				else:
					next_layer_id = -1
					
			elif crossing_state == 2:
				current_region = None
				next_regions = []
				inLast = True
				inGraph = False
				next_layer_id = -1
					
					
			front_region = get_front_region(self,ped_p,next_regions,self.front_v[i],self.right_v[i])			
			target_Region,Region_value = Caculate_and_Get_target_Region(self,i,G,next_regions,ped_p,current_region,self.front_v[i],self.right_v[i],goal_sidewalk_region)			
			next_point,moving_around = go_to_next_target(self,current_region,target_Region,front_region,ped_p,i,inLast,self.front_v[i],self.right_v[i],goal_sidewalk_region)
			
			#if moving_around:
			#	self.stop[i] = True
			
			if current_region != None:								   
				unsafe = False			  
				collision_avoidance(self,inLast,inGraph,moving_around,unsafe,i,current_region,ped_p,self.front_v[i],self.right_v[i])

			for layer_count,lanes_array_for_get_leader in enumerate(self.veh_env.lanes_array_for_get_leader):
				if layer_count == in_layer_id or layer_count == next_layer_id and self.stop[i] == False:		 
					if not ('ped',i) in lanes_array_for_get_leader:
						lanes_array_for_get_leader.append(('ped',i))
				elif layer_count == in_layer_id and self.stop[i] == True:		 
					if not ('ped',i) in lanes_array_for_get_leader:
						lanes_array_for_get_leader.append(('ped',i))
				else:
					if ('ped',i) in lanes_array_for_get_leader:
						lanes_array_for_get_leader.remove(('ped',i))


			self.state[i, 4:6] = np.array([next_point.x,next_point.y])
			self.save_data(crossing_state,i)