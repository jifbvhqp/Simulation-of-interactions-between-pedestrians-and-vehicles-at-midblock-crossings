import argparse
import glob
import os
import sys
import time
import numpy as np
import math
import random
from random import randrange
import argparse
import threading
import time
from tooluse import get_Node_Width_Height,CellCollision,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
import csv
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
import pygame
import shutil
import psutil
import matplotlib
import matplotlib.pyplot as plt 
from carla_world import world
def kill_server():
	read_pid = -1
	with open('unreal_pid.txt') as f:
		lines = f.readlines()
		unreal_pid = int(lines[0])
	
	pids = psutil.pids()

	server_pid = -1
	for pid in pids:
		p = psutil.Process(pid)
		if p.name() == 'UE4Editor.exe' and unreal_pid != pid:
			server_pid = pid
			break
			
	p = psutil.Process(server_pid)
	p.terminate()
def get_agents_num_and_total_num(filepath):

	temp = []
	total_frame = 0
	agents_num = 0
	
	with open(filepath, newline='') as csvfile:
		rows = csv.reader(csvfile)
		for count,row in enumerate(rows):
			if count > 0:
				temp.append(row)

				if total_frame < int(row[0]):
					total_frame = int(row[0])
				if agents_num < int(row[1]):
					agents_num = int(row[1])
					
	return agents_num,total_frame,temp
def get_agent_first_appear(temp,agent_nums,shift = False):
	first_appear = [-1 for j in range(agent_nums)]
	for row in temp:
		index = int(row[1]) - 1 if shift else int(row[1])
		if first_appear[index] == -1:
			first_appear[index] = int(int(row[0]))
	return first_appear			
def read_ped_csv(filepath):	
	agents_num,total_frame,temp = get_agents_num_and_total_num(filepath)

	not_to_end = [True for j in range(agents_num+1)]
	first_appear = get_agent_first_appear(temp,agents_num+1,shift = False)
	#x , y , state , direction , alive ,vx, vy?
	frames = [[[-9999,-9999,-1,-1,0,-9999,-9999] for j in range(agents_num+1)] for i in range(total_frame+10)]
	for row in temp:
		if int(row[2]) == 2:
			not_to_end[int(row[1])] = False
		frames[int(row[0])][int(row[1])][0] = float(row[4])
		frames[int(row[0])][int(row[1])][1] = float(row[5])
		frames[int(row[0])][int(row[1])][2] = int(row[2])
		frames[int(row[0])][int(row[1])][3] = int(row[3])
		frames[int(row[0])][int(row[1])][4] = 1	
		frames[int(row[0])][int(row[1])][5] = float(row[6])
		frames[int(row[0])][int(row[1])][6] = float(row[7])
	return frames,not_to_end,first_appear	
def read_veh_csv(filepath):	
	agents_num,total_frame,temp = get_agents_num_and_total_num(filepath)
	first_appear = get_agent_first_appear(temp,agents_num+1,shift = False)
	#x , y , acc , yaw angle ,blueprint, alive ?
	frames = [[[-9999,-9999,-9999,-9999,'',0] for j in range(agents_num+1)] for i in range(total_frame+10)]
	
	for row in temp:
		frames[int(row[0])][int(row[1])][0] = float(row[2])
		frames[int(row[0])][int(row[1])][1] = float(row[3])
		frames[int(row[0])][int(row[1])][2] = float(row[7])
		frames[int(row[0])][int(row[1])][3] = float(row[9])
		frames[int(row[0])][int(row[1])][4] = row[10]
		frames[int(row[0])][int(row[1])][5] = 1
	return frames,first_appear	
def destroy_all_agents(pedestrians,vehicles):
	for agent in pedestrians:
		if agent != None:
			agent.destroy()
	
	for agent in vehicles:
		if agent != None:
			agent.destroy()
def agent_is_active(appear_frame_num,current_frame_num,active_code,agent):
	return appear_frame_num < current_frame_num and active_code == 1 and agent != None
def need_spawn_agent(appear_frame_num,current_frame_num,active_code,agent):	 
	return appear_frame_num == current_frame_num and active_code == 1 and agent == None
def need_destroy_agent(appear_frame_num,current_frame_num,total_appear_frame_num,active_code,agent):	 
	return (appear_frame_num < current_frame_num and (active_code == 0 or total_appear_frame_num == current_frame_num)) and agent != None
def spawn_agent(bp,loc,rot,v):
	agent = None
	while agent == None:		  
		agent = v.carla_world.try_spawn_actor(bp,carla.Transform(loc,rot))
		loc.z += 1
	return agent
def ped_move(pedestrian,move_dir,loc,fc,ped_speed):
	pedestrian.apply_control(carla.WalkerControl(move_dir,ped_speed,False))
	pedestrian.set_location(loc)
def save_datas(all_datas_num,frame_datas,path):	
	
	if not os.path.exists('D://'+path):
		try:
			os.makedirs('D://'+path)
		except FileExistsError:
			pass
	
	for j in range(all_datas_num):
		count = 1
		filepath = 'D://'+path+'//'+str(count)
		while os.path.exists(filepath):
			count += 1
			filepath = 'D://'+path+'//'+str(count)
		try:
			os.makedirs(filepath)
		except FileExistsError:
			pass
		for c in range(0,len(frame_datas[j])):
			frame_datas[j][c][0].save_to_disk(filepath + '//' + str(c+1) +'.png')
		#for c,image in enumerate(frame_datas[j]):
		#	image[0].save_to_disk(filepath + '//' + str(c+1) +'.png')
		print(str(j) + 'saved')			
def replay_animation(ped_datas,veh_datas,path,weather,no_render_mode = False,save_image = False):	
	try:
		cnt = 0
		frame_datas = []
		for ped_data,veh_data in zip(ped_datas,veh_datas):
			ped_frames,_,ped_first_appear = read_ped_csv(ped_data)			
			veh_frames,veh_first_appear = read_veh_csv(veh_data)
			total_frame = len(ped_frames) if len(ped_frames) > len(veh_frames) else len(veh_frames)

			image_array = []
			pedestrians = [None for i in range(len(ped_frames[0]))] if len(ped_frames) > 0 else []
			vehicles = [None for i in range(len(veh_frames[0]))] if len(veh_frames) > 0 else []	 
			v = world(0,weather,path,draw_mode = False,simulate_mode = False,no_render_mode = no_render_mode)			

			for fc in range(total_frame):
				if len(ped_frames) > 0 and fc < len(ped_frames):
					for pedid in range(len(ped_frames[fc])):
						
						#if pedid == 0 or pedid == 19:
						#	continue
							
						#if pedid == 28 or pedid == 29 or pedid == 37:
						#	continue
							
						#scenario_e
						#if pedid == 25 or pedid == 6 or pedid == 23 or pedid == 26 or pedid == 4 or pedid == 24:# or pedid == 10 or pedid == 14 or pedid == 21 or pedid == 22 or pedid == 19:
						#	continue
						
						alive_code = ped_frames[fc][pedid][4]
						appear_frame_num = ped_first_appear[pedid]
						if need_spawn_agent(appear_frame_num,fc,alive_code,pedestrians[pedid]):
							pedestrians[pedid] = spawn_agent(random.choice(v.blueprintsWalkers),carla.Location(ped_frames[fc][pedid][0],ped_frames[fc][pedid][1],1),carla.Rotation(0,0,0),v)
			
						if agent_is_active(appear_frame_num,fc,alive_code,pedestrians[pedid]):					
							
							ped_speed = math.sqrt(ped_frames[fc][pedid][5]**2 + ped_frames[fc][pedid][6]**2)
							if fc+1 < len(ped_frames):
								ped_move(pedestrians[pedid],normalize(carla.Location(ped_frames[fc+1][pedid][0],ped_frames[fc+1][pedid][1],0) - carla.Location(ped_frames[fc][pedid][0],ped_frames[fc][pedid][1],0)),carla.Location(ped_frames[fc][pedid][0],ped_frames[fc][pedid][1],1),fc,ped_speed)
			
						if need_destroy_agent(appear_frame_num,fc,len(ped_frames),alive_code,pedestrians[pedid]):
							pedestrians[pedid].destroy()
							pedestrians[pedid] = None
								
						v.draw_string(carla.Location(ped_frames[fc][pedid][0],ped_frames[fc][pedid][1],2.0),-1,str(pedid),carla.Color(255,0,0))
				
				if len(veh_frames) > 0:
					for vehid in range(len(veh_frames[fc])):
						alive_code = veh_frames[fc][vehid][5]
						appear_frame_num = veh_first_appear[vehid]
	
						if need_spawn_agent(appear_frame_num,fc,alive_code,vehicles[vehid]):
							bp = random.choice([x for x in v.carla_world.get_blueprint_library().filter(veh_frames[fc][vehid][4])])
							loc = carla.Location(veh_frames[fc][vehid][0],veh_frames[fc][vehid][1],1)
							rot = carla.Rotation(0,veh_frames[fc][vehid][3],0)
							vehicles[vehid] = spawn_agent(bp,loc,rot,v)
							vehicles[vehid].set_simulate_physics(False)
						if agent_is_active(appear_frame_num,fc,alive_code,vehicles[vehid]):	
							vehicles[vehid].set_transform(carla.Transform(carla.Location(veh_frames[fc][vehid][0],veh_frames[fc][vehid][1],0),carla.Rotation(0,veh_frames[fc][vehid][3],0)))	
								
							#if veh_frames[fc][vehid][7]:
							#	v.draw_point(carla.Location(veh_frames[fc][vehid][0],veh_frames[fc][vehid][1],3.0),1/29,carla.Color(255,0,0),size=0.1)
							#else:
							#	v.draw_point(carla.Location(veh_frames[fc][vehid][0],veh_frames[fc][vehid][1],3.0),1/29,carla.Color(0,255,0),size=0.1)
							
						if need_destroy_agent(appear_frame_num,fc,len(veh_frames),alive_code,vehicles[vehid]):
							vehicles[vehid].destroy()
							vehicles[vehid] = None
				
				if not save_image:
					v.carla_world.tick()
				else:
					image_rgb = v.tick(timeout=2.0)
					image_array.append(image_rgb)
		
			v.destroy()
			destroy_all_agents(pedestrians,vehicles)
			if save_image:		  
				frame_datas.append(image_array)
			cnt += 1
		if save_image:
			save_datas(len(ped_datas),frame_datas,path)
			
	except KeyboardInterrupt:
		v.destroy()
		destroy_all_agents(pedestrians,vehicles)
	finally:
		v.destroy()
		destroy_all_agents(pedestrians,vehicles)
		#os.system('shutdown -s')