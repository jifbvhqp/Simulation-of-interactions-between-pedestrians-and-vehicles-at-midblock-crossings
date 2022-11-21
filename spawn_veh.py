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
def pre_spawn_motor(veh_ev,spawn_loc,spawn_yaw,spawn_layer):
	bp = random.choice(veh_ev.world.blueprintsMotorcycles)
	vehicle = veh_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(spawn_loc,carla.Rotation(0,spawn_yaw,0)))
	
	if vehicle != None:
		vehicle.set_simulate_physics(False)
		vehicle.set_location(carla.Location(spawn_loc.x,spawn_loc.y,0.0))
		
		rearRaduis,leftRaduis = 1.1089935302734375,0.433537956882649
		veh_ev.veh_rear.append(rearRaduis)
		veh_ev.veh_raduis.append(leftRaduis)	 
		veh_ev.bprint.append(bp)	
					   
		veh_ev.velo_i.append(0)
		veh_ev.yaw_rate.append(0)
		veh_ev.loc_i.append(carla.Location(spawn_loc.x,spawn_loc.y,0.0))
		veh_ev.yaw_i.append(spawn_yaw)
		veh_ev.a_idm.append(0)
		
		veh_ev.min_gap.append(random.uniform(veh_ev.para_random[0][0],veh_ev.para_random[0][1]))			 
		veh_ev.comb.append(random.uniform(veh_ev.para_random[1][0],veh_ev.para_random[1][1]))
		veh_ev.maxa.append(random.uniform(veh_ev.para_random[2][0],veh_ev.para_random[2][1]))
		veh_ev.desireS.append(random.uniform(veh_ev.para_random[3][0],veh_ev.para_random[3][1]))			   
		
		veh_ev.lanechanging.append(False)
		veh_ev.lane_change_phase.append(0)
		veh_ev.target_yaw.append(0)
		veh_ev.sign.append(1)
		veh_ev.return_proj_vec.append(carla.Vector3D(0,0,0))
		veh_ev.driver_lane_change_info.append(None)
		veh_ev.lanechange_side.append(None)
		veh_ev.have_changed.append(False)
		veh_ev.blocked_idx.append(-1)
		veh_ev.vehicles.append(vehicle)
		
		veh_ev.lanes_array_for_get_leader[spawn_layer].append(veh_ev.veh_count)
		veh_ev.lanes_array[spawn_layer].append(veh_ev.veh_count)
		veh_ev.veh_count += 1
def pre_spawn_car(veh_ev,spawn_loc,spawn_yaw,spawn_layer1,spawn_layer2):
	
	if veh_ev.save_folder == 'scenario_b':
		bp = random.choice(veh_ev.world.blueprintsSameCar)

		if bp.has_attribute('color'):
			color = random.choice(bp.get_attribute('color').recommended_values)
			color = '255,255,255'
			bp.set_attribute('color', color)	
	else:
		bp = random.choice(veh_ev.world.blueprintsVehicles)
	
	vehicle = veh_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(carla.Location(spawn_loc.x,spawn_loc.y,1.0),carla.Rotation(0,spawn_yaw,0)))	
	
	if vehicle != None:
		vehicle.set_simulate_physics(False)
		vehicle.set_location(carla.Location(spawn_loc.x,spawn_loc.y,0.0))
		
		rearRaduis,leftRaduis = get_car_rear(vehicle)
		veh_ev.veh_rear.append(rearRaduis)
		veh_ev.veh_raduis.append(leftRaduis)	 
		veh_ev.bprint.append(bp)	
					   
		veh_ev.velo_i.append(0)
		veh_ev.yaw_rate.append(0)
		veh_ev.loc_i.append(carla.Location(spawn_loc.x,spawn_loc.y,0.0))
		veh_ev.yaw_i.append(spawn_yaw)
		veh_ev.a_idm.append(0)
		
		veh_ev.min_gap.append(random.uniform(veh_ev.para_random[0][0],veh_ev.para_random[0][1]))			 
		veh_ev.comb.append(random.uniform(veh_ev.para_random[1][0],veh_ev.para_random[1][1]))
		veh_ev.maxa.append(random.uniform(veh_ev.para_random[2][0],veh_ev.para_random[2][1]))
		veh_ev.desireS.append(random.uniform(veh_ev.para_random[3][0],veh_ev.para_random[3][1]))				
		
		veh_ev.lanechanging.append(False)
		veh_ev.lane_change_phase.append(0)
		veh_ev.target_yaw.append(0)
		veh_ev.sign.append(1)
		veh_ev.return_proj_vec.append(carla.Vector3D(0,0,0))
		veh_ev.driver_lane_change_info.append(None)
		veh_ev.lanechange_side.append(None)
		veh_ev.have_changed.append(False)
		veh_ev.blocked_idx.append(-1)
		veh_ev.vehicles.append(vehicle)
		
		veh_ev.lanes_array[spawn_layer1].append(veh_ev.veh_count)
		veh_ev.lanes_array[spawn_layer2].append(veh_ev.veh_count)
		
		veh_ev.lanes_array_for_get_leader[spawn_layer1].append(veh_ev.veh_count)
		veh_ev.lanes_array_for_get_leader[spawn_layer2].append(veh_ev.veh_count)
		veh_idx = veh_ev.veh_count
		veh_ev.veh_count += 1
			
	return veh_idx
def spawn_car(veh_ev,bp=None,spawn_trs=None,blocking_veh = False):
	veh_idx = -1
	if bp == None and spawn_trs == None:
		bp = random.choice(veh_ev.world.blueprintsVehicles)
		randIdx = random.randint(0,len(veh_ev.car_spawn_points))
		spawn_trs = veh_ev.car_spawn_points[randIdx]
		
	va_enough_far = False
	vb_enough_far = False
	if len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]]) > 0:
		if isinstance(veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1],int):
			if veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1]].is_alive:
				va = veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1]]
				if va.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					va_enough_far = True
		else:
			if veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1][1]].is_alive:
				va = veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1][1]]
				if va.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					va_enough_far = True
	else:
		va_enough_far = True
	
	if len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]]) > 0:
		if isinstance(veh_ev.lanes_array_for_get_leader[spawn_trs[3]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]])-1],int):
			if veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[3]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]])-1]].is_alive:
				vb = veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[3]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]])-1]]
				if vb.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					vb_enough_far = True	
		else:
			if veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[3]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]])-1][1]].is_alive:
				vb = veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[3]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[3]])-1][1]]
				if vb.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					vb_enough_far = True
	else:
		vb_enough_far = True
	
	
	if blocking_veh:
		va_enough_far = True
		vb_enough_far = True
	
	
	if va_enough_far and vb_enough_far:
		vehicle = veh_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(carla.Location(spawn_trs[0].x,spawn_trs[0].y,1.0),carla.Rotation(0,spawn_trs[1],0)))	
		
		if vehicle != None:
			vehicle.set_simulate_physics(False)
			vehicle.set_location(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0))
			
			rearRaduis,leftRaduis = get_car_rear(vehicle)
			veh_ev.veh_rear.append(rearRaduis)
			veh_ev.veh_raduis.append(leftRaduis)	 
			veh_ev.bprint.append(bp)	
						   
			veh_ev.velo_i.append(0)
			veh_ev.yaw_rate.append(0)
			veh_ev.loc_i.append(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0))
			veh_ev.yaw_i.append(spawn_trs[1])
			veh_ev.a_idm.append(0)
			
			veh_ev.min_gap.append(random.uniform(veh_ev.para_random[0][0],veh_ev.para_random[0][1]))			 
			veh_ev.comb.append(random.uniform(veh_ev.para_random[1][0],veh_ev.para_random[1][1]))
			veh_ev.maxa.append(random.uniform(veh_ev.para_random[2][0],veh_ev.para_random[2][1]))
			veh_ev.desireS.append(random.uniform(veh_ev.para_random[3][0],veh_ev.para_random[3][1]))				
			
			veh_ev.lanechanging.append(False)
			veh_ev.lane_change_phase.append(0)
			veh_ev.target_yaw.append(0)
			veh_ev.sign.append(1)
			veh_ev.return_proj_vec.append(carla.Vector3D(0,0,0))
			veh_ev.driver_lane_change_info.append(None)
			veh_ev.lanechange_side.append(None)
			veh_ev.have_changed.append(False)
			veh_ev.blocked_idx.append(-1)
			veh_ev.vehicles.append(vehicle)
			
			veh_ev.lanes_array[spawn_trs[2]].append(veh_ev.veh_count)
			veh_ev.lanes_array[spawn_trs[3]].append(veh_ev.veh_count)
			
			veh_ev.lanes_array_for_get_leader[spawn_trs[2]].append(veh_ev.veh_count)
			veh_ev.lanes_array_for_get_leader[spawn_trs[3]].append(veh_ev.veh_count)
			veh_idx = veh_ev.veh_count
			veh_ev.veh_count += 1
			if veh_ev.save_folder == 'scenario_g':
				if spawn_trs[2] < 7 and spawn_trs[3] < 7:
					veh_ev.veh_dir.append(0)
				else:
					veh_ev.veh_dir.append(1)
	return veh_idx		
def spawn_motor(veh_ev):
	
	#if bp == None and spawn_trs == None:
	bp = random.choice(veh_ev.world.blueprintsMotorcycles)
	randIdx = random.randint(0,len(veh_ev.motor_spawn_points))
	spawn_trs = veh_ev.motor_spawn_points[randIdx]
	
	va_enough_far = False
	if len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]]) > 0:
		if isinstance(veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1],int):
			if veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1]].is_alive:
				va = veh_ev.vehicles[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1]]
				if va.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					va_enough_far = True
		else:
			if veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1][1]].is_alive:
				va = veh_ev.ped_env.pedestrians[veh_ev.lanes_array_for_get_leader[spawn_trs[2]][len(veh_ev.lanes_array_for_get_leader[spawn_trs[2]])-1][1]]
				if va.get_location().distance(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0)) > 8:
					va_enough_far = True
	else:
		va_enough_far = True
	
	#if blocking_veh:
	#	va_enough_far = True

	if va_enough_far:
		vehicle = veh_ev.world.carla_world.try_spawn_actor(bp,carla.Transform(spawn_trs[0],carla.Rotation(0,spawn_trs[1],0)))
		
		if vehicle != None:
			vehicle.set_simulate_physics(False)
			vehicle.set_location(carla.Location(spawn_trs[0].x,spawn_trs[0].y,0.0))
			
			rearRaduis,leftRaduis = 1.1089935302734375,0.433537956882649
			veh_ev.veh_rear.append(rearRaduis)
			veh_ev.veh_raduis.append(leftRaduis)	 
			veh_ev.bprint.append(bp)	
						   
			veh_ev.velo_i.append(0)
			veh_ev.yaw_rate.append(0)
			veh_ev.loc_i.append(spawn_trs[0])
			veh_ev.yaw_i.append(spawn_trs[1])
			veh_ev.a_idm.append(0)
			
			veh_ev.min_gap.append(random.uniform(veh_ev.para_random[0][0],veh_ev.para_random[0][1]))			 
			veh_ev.comb.append(random.uniform(veh_ev.para_random[1][0],veh_ev.para_random[1][1]))
			veh_ev.maxa.append(random.uniform(veh_ev.para_random[2][0],veh_ev.para_random[2][1]))
			veh_ev.desireS.append(random.uniform(veh_ev.para_random[3][0],veh_ev.para_random[3][1]))			   
			
			veh_ev.lanechanging.append(False)
			veh_ev.lane_change_phase.append(0)
			veh_ev.target_yaw.append(0)
			veh_ev.sign.append(1)
			veh_ev.return_proj_vec.append(carla.Vector3D(0,0,0))
			veh_ev.driver_lane_change_info.append(None)
			veh_ev.lanechange_side.append(None)
			veh_ev.have_changed.append(False)
			veh_ev.blocked_idx.append(-1)
			veh_ev.vehicles.append(vehicle)
			
			veh_ev.lanes_array_for_get_leader[spawn_trs[2]].append(veh_ev.veh_count)
			veh_ev.lanes_array[spawn_trs[2]].append(veh_ev.veh_count)
			veh_ev.veh_count += 1
			
			if veh_ev.save_folder == 'scenario_g':
				if spawn_trs[2] < 7:
					veh_ev.veh_dir.append(0)
				else:
					veh_ev.veh_dir.append(1)