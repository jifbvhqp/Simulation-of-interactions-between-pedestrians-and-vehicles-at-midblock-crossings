import glob
import os
import sys
import math
import random
from tooluse import get_car_rear,get_Node_Width_Height,CellCollision,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
import argparse
from numpy import random
import numpy as np

def spawn_player_pedestrian(ped_ev,spawn_loc,ped_type,goali):
	
	bp = random.choice(ped_ev.world.blueprintsWalkers)	
	pedestrian = ped_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(carla.Location(spawn_loc.x,spawn_loc.y,1.0),carla.Rotation(0,135,0)))	
	
	idx = -1
	
	if pedestrian != None:	
		front_vec = carla.Vector3D(0,0,0)
		right_vec = carla.Vector3D(0,0,0)
		
		for v in ped_ev.ped_direction:
			if ped_type == v[2]:
				front_vec = v[0]
				right_vec = v[1]
				break
		
		if ped_ev.size() > 0:
			ped_ev.state = np.vstack((ped_ev.state,[[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]]))  
		else: 
			ped_ev.state = np.array([[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]])
			
		
		ped_ev.pedestrians.append(pedestrian)
		idx = len(ped_ev.pedestrians) - 1
		
		ped_ev.max_speed_multiplier.append(random.uniform(ped_ev.speed_under,ped_ev.speed_up))
		ped_ev.initial_speeds = np.ones((ped_ev.size()))
		ped_ev.max_speeds = ped_ev.max_speed_multiplier * ped_ev.initial_speeds
		
		ped_ev.front_v.append(front_vec)
		ped_ev.right_v.append(right_vec)
			
		ped_ev.stop.append(False)
		ped_ev.close_move_around_vehicle.append(random.uniform(1.0,3.0))
		ped_ev.ped_type.append(ped_type)
		ped_ev.ped_goal.append(goali)
		ped_ev.temp_goal.append([])
		ped_ev.temp_type.append([3])
		ped_ev.in_layer_id.append(-1)
		ped_ev.player_ped_idxs.append(idx)
		#pedestrian.set_simulate_physics(False)
	
	return idx	
	
def spawn_pedestrian2(ped_ev,spawn_loc,init_type,temp_goal,temp_ped_type):
	
	bp = random.choice(ped_ev.world.blueprintsWalkers)

	spawn_loc = spawn_loc
	ped_type = init_type
	goali = temp_goal[0]
	
	pedestrian = ped_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(carla.Location(spawn_loc.x,spawn_loc.y,1.0),carla.Rotation(0,0,0)))	
	
	if pedestrian != None:	
		front_vec = carla.Vector3D(0,0,0)
		right_vec = carla.Vector3D(0,0,0)
		
		for v in ped_ev.ped_direction:
			if ped_type == v[2]:
				front_vec = v[0]
				right_vec = v[1]
				break

		if ped_ev.size() > 0:
			ped_ev.state = np.vstack((ped_ev.state,[[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]]))  
		else: 
			ped_ev.state = np.array([[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]])

		ped_ev.pedestrians.append(pedestrian)
		

		ped_ev.max_speed_multiplier.append(random.uniform(ped_ev.speed_under,ped_ev.speed_up))	
		ped_ev.initial_speeds = np.ones((ped_ev.size()))
		ped_ev.max_speeds = ped_ev.max_speed_multiplier * ped_ev.initial_speeds
		
		ped_ev.front_v.append(front_vec)
		ped_ev.right_v.append(right_vec)
			
		ped_ev.stop.append(False)
		ped_ev.close_move_around_vehicle.append(random.uniform(1.0,3.0))
		ped_ev.ped_type.append(ped_type)
		ped_ev.ped_goal.append(goali)
		ped_ev.temp_goal.append(temp_goal)
		ped_ev.temp_type.append(temp_ped_type)
		
		pedestrian.set_simulate_physics(False)
		
def spawn_pedestrian(ped_ev):
	
	bp = random.choice(ped_ev.world.blueprintsWalkers)	
	randIdx = random.randint(0,len(ped_ev.ped_spawn_point))
	
	spawn_loc = ped_ev.ped_spawn_point[randIdx][0]
	ped_type = ped_ev.ped_spawn_point[randIdx][1]
	goali = ped_ev.ped_spawn_point[randIdx][2]
	
	pedestrian = ped_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(carla.Location(spawn_loc.x,spawn_loc.y,1.0),carla.Rotation(0,0,0)))	
	
	if pedestrian != None:	
		front_vec = carla.Vector3D(0,0,0)
		right_vec = carla.Vector3D(0,0,0)
		
		for v in ped_ev.ped_direction:
			if ped_type == v[2]:
				front_vec = v[0]
				right_vec = v[1]
				break
				
		if ped_ev.size() > 0:
			ped_ev.state = np.vstack((ped_ev.state,[[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]]))  
		else: 
			ped_ev.state = np.array([[spawn_loc.x,spawn_loc.y,front_vec.x,front_vec.y,goali.x,goali.y]])

		ped_ev.pedestrians.append(pedestrian)
		
		ped_ev.max_speed_multiplier.append(random.uniform(ped_ev.speed_under,ped_ev.speed_up))
		ped_ev.initial_speeds = np.ones((ped_ev.size()))
		ped_ev.max_speeds = ped_ev.max_speed_multiplier * ped_ev.initial_speeds
		
		ped_ev.front_v.append(front_vec)
		ped_ev.right_v.append(right_vec)
			
		ped_ev.stop.append(False)
		ped_ev.close_move_around_vehicle.append(random.uniform(1.0,3.0))
		ped_ev.ped_type.append(ped_type)
		ped_ev.ped_goal.append(goali)
		ped_ev.temp_goal.append([])
		ped_ev.temp_type.append([3])
		
		pedestrian.set_simulate_physics(False)