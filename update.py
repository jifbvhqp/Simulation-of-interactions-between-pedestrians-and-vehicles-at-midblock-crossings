import argparse
import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
import matplotlib.pyplot as plt 
from random import randrange
import argparse
from carla_world import world
import threading
import time
import csv
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
from tooluse import isInRect
import carla
import pygame
import shutil
import psutil
from create_graph import graph
from veh_update import veh_env
from ped_update import ped_env
from Keyboard import Keyboard
from spawn_veh import spawn_car
from spawn_ped import spawn_player_pedestrian
def write_csv(filename,header,data):
	with open(filename, 'w',newline="", encoding='UTF8') as f:
		writer = csv.writer(f)
		writer.writerow(header)
		# write the data
		writer.writerows(data)
def save_simulate_data(save_folder,ped_datas,veh_datas):
	filepath = save_folder 
	if not os.path.exists(filepath):
		try:
			os.makedirs(filepath)
		except FileExistsError:
			pass

	for i,data in enumerate(ped_datas):		
		filepath = save_folder+'/'+str(i+1)
		if not os.path.exists(filepath):
			try:
				os.makedirs(filepath)
			except FileExistsError:
				pass
		else:
			count = i+2
			filepath = save_folder+'/'+str(count)
			while os.path.exists(filepath):
				filepath = save_folder+'/'+str(count)
				count += 1
			try:
				os.makedirs(filepath)
			except FileExistsError:
				pass

		#ped_data
		header = ['frame','ped_id','state','ped_type','x','y','vx','vy','w1','w2','w3','w4','w5']
		filename = filepath+'/'+'ped.csv'				
		write_csv(filename,header,ped_datas[i])


		#veh data
		header = ['frame','veh_id','vehx','vehy','veh_width','veh_height','veh_speed','acc','leader_type','yaw','bp']
		filename = filepath+'/'+'veh.csv'				
		write_csv(filename,header,veh_datas[i])
def parse_args():
	parser = argparse.ArgumentParser() 
	parser.add_argument('--count', '-c', default='1', type=int, required=False, help='parameters file count')
	parser.add_argument('--ped_num', '-n', default='50', type=int, required=False, help='ped_num')
	parser.add_argument('--render_mode', '-r', default=False, type=bool, required=False, help='render_mode')
	parser.add_argument('--shut_down', '-s', default=False, type=bool, required=False, help='shut_down')
	return parser.parse_args()	  
def draw_rect(v,p1,p2,p3,p4,color):
	p1.z,p2.z,p3.z,p4.z = 0.2,0.2,0.2,0.2
	v.world.draw_line(p1,p2,1/29,color)
	v.world.draw_line(p2,p3,1/29,color)
	v.world.draw_line(p3,p4,1/29,color)
	v.world.draw_line(p4,p1,1/29,color)
def read_csv(filepath):
	temp = []
	with open(filepath, newline='') as csvfile:
		rows = csv.reader(csvfile)
		for count,row in enumerate(rows):
			if count > 0:		  
				temp.append(row)
	return temp	   
def get_para_data(para_path):
	basic_para_rows = read_csv(para_path)[0]
	veh_arrival_rate = -1
	total_frame = -1
	min_ped_speed = 0.0
	max_ped_speed = 0.0
	min_veh_desire_speed = 0.0
	max_veh_desire_speed = 0.0
	w1,w2,w3,w4,w5 = 0.0,0.0,0.0,0.0,0.0

	veh_arrival_rate = int(basic_para_rows[0])
	total_frame = int(basic_para_rows[1])
	min_ped_speed = float(basic_para_rows[2])
	max_ped_speed = float(basic_para_rows[3])
	min_veh_desire_speed = float(basic_para_rows[4])
	max_veh_desire_speed = float(basic_para_rows[5])
	w1,w2,w3,w4,w5 = float(basic_para_rows[6]),float(basic_para_rows[7]),float(basic_para_rows[8]),float(basic_para_rows[9]),float(basic_para_rows[10])
		
	return veh_arrival_rate,total_frame,min_ped_speed,max_ped_speed,min_veh_desire_speed,max_veh_desire_speed,w1,w2,w3,w4,w5
def get_world_data(spawn_end_path,weather_path):
	
	spawn_end_rows = read_csv(spawn_end_path)
	weather_row = read_csv(weather_path)

	vehicles_spawn_end_points = []
	weather = ''
	
	#spawnx,spawny,endx,endy,spawn_yaw
	for row in spawn_end_rows:
		vehicles_spawn_end_points.append((carla.Location(float(row[0]),float(row[1]),0.0),carla.Location(float(row[2]),float(row[3]),0.0),carla.Location(float(row[4]),float(row[5]),0.0),carla.Location(float(row[6]),float(row[7]),0.0)))
	
	weather = weather_row[0][0]
	
	return vehicles_spawn_end_points,weather
def get_graph_data(graph_bottom_path,car_spawn_points_path,motor_spawn_points_path,motor_pre_spawn_path,car_pre_spawn_path):
	graph_bottom_row = read_csv(graph_bottom_path)
	car_spawn_points_rows = read_csv(car_spawn_points_path)
	motor_spawn_points_rows = read_csv(motor_spawn_points_path)
	motor_pre_spawn_rows = read_csv(motor_pre_spawn_path)
	car_pre_spawn_rows = read_csv(car_pre_spawn_path)
	
	car_spawn_points = []
	motor_spawn_points = []
	graph_attr = []
	motor_pre_spawn = []
	car_pre_spawn = []
	
	#leftx,lefty
	#rightx,righty
	for row in graph_bottom_row:
		graph_attr.append(carla.Location(float(row[0]),float(row[1]),0.0))
		graph_attr.append(carla.Location(float(row[2]),float(row[3]),0.0))	
		graph_attr.append(carla.Vector3D(float(row[4]),float(row[5]),0.0))
		graph_attr.append(int(row[6]))
		graph_attr.append(float(row[7]))
		graph_attr.append(carla.Vector3D(float(row[8]),float(row[9]),0.0))

	#spawnx,spawny,spawn_yaw
	for row in car_spawn_points_rows:
		car_spawn_points.append((carla.Location(float(row[0]),float(row[1]),0.0),float(row[2]),int(row[3]),int(row[4])))
		
	#spawnx,spawny,spawn_yaw
	for row in motor_spawn_points_rows:
		motor_spawn_points.append((carla.Location(float(row[0]),float(row[1]),0.0),float(row[2]),int(row[3])))
	
	for row in motor_pre_spawn_rows:
		motor_pre_spawn.append((carla.Location(float(row[0]),float(row[1]),0.0),float(row[2]),int(row[3]),int(row[4])))
	
	for row in car_pre_spawn_rows:
		car_pre_spawn.append((carla.Location(float(row[0]),float(row[1]),0.0),float(row[2]),int(row[3]),int(row[4]),int(row[5])))
	
	return car_spawn_points,motor_spawn_points,graph_attr,motor_pre_spawn,car_pre_spawn
def get_sidewalk_data(ped_spawns_points_path,ped_side_walk_path,ped_direction_path,ped_spawn_end_points_path):
	
	ped_spawn_points_rows = read_csv(ped_spawns_points_path)
	ped_side_walk_rows = read_csv(ped_side_walk_path)
	ped_direction_rows = read_csv(ped_direction_path)
	ped_spawn_end_rows = read_csv(ped_spawn_end_points_path)
	
	ped_spawn_points = []
	ped_side_walk = []
	ped_direction = []
	ped_spawn_end_points = []
	
	for row in ped_spawn_points_rows:
		ped_spawn_points.append((carla.Location(float(row[1]),float(row[2]),1.0),int(row[0]),carla.Location(float(row[3]),float(row[4]),1.0)))
	for row in ped_side_walk_rows:
		print(row)
		ped_side_walk.append(((carla.Location(float(row[1]),float(row[2]),0.0),carla.Location(float(row[3]),float(row[4]),0.0),carla.Location(float(row[5]),float(row[6]),0.0),carla.Location(float(row[7]),float(row[8]),0.0)),int(row[0])))	
	for row in ped_direction_rows:
		ped_direction.append((carla.Vector3D(float(row[1]),float(row[2]),0.0),carla.Vector3D(float(row[3]),float(row[4]),0.0),int(row[0])))
	
	for row in ped_spawn_end_rows:
		temp_goal = []
		temp_type = []
		tmprowidx = 6
		for i in range(int(row[4])):
			temp_goal.append(carla.Location(float(row[tmprowidx]),float(row[tmprowidx+1]),0.0))
			tmprowidx += 2
		tmprowidx = 6+2*int(row[4])
		for i in range(int(row[5])):
			temp_type.append(int(row[tmprowidx]))
			tmprowidx += 1	
		ped_spawn_end_points.append((carla.Location(float(row[0]),float(row[1]),0.0),int(row[2]),int(row[3]),temp_goal,temp_type))
		
	
	return ped_spawn_points,ped_side_walk,ped_direction,ped_spawn_end_points
def get_rect(loc,r,f,rect_width,rect_height):
	rect = lane_point_gen_rect(loc,r,f,rect_width,rect_height)	 
	return rect


save_folder = read_csv('scenario.csv')[0][0]
spawn_end_path,weather_path = 'vehdatas/'+save_folder+'/spawn_end.csv','vehdatas/'+save_folder+'/weather.csv'
car_spawn_points_path,motor_spawn_points_path,graph_bottom_path,motor_pre_spawn_path,car_pre_spawn_path = 'vehdatas/'+save_folder+'/car_spawn_points.csv','vehdatas/'+save_folder+'/motor_spawn_points.csv','vehdatas/'+save_folder+'/graph_bottom_coord.csv','vehdatas/'+save_folder+'/motor_pre_spawn.csv','vehdatas/'+save_folder+'/car_pre_spawn.csv'
ped_spawns_points_path,ped_side_walk_path = 'peddatas/'+save_folder+'/ped_spawn_points.csv','peddatas/'+save_folder+'/sidewalk.csv'
ped_direction_path,ped_spawn_end_points_path = 'peddatas/'+save_folder+'/ped_direction.csv','peddatas/'+save_folder+'/ped_spawn_end_points.csv'

vehicles_spawn_end_points,weather = get_world_data(spawn_end_path,weather_path)
car_spawn_points,motor_spawn_points,graph_attr,motor_pre_spawn,car_pre_spawn = get_graph_data(graph_bottom_path,car_spawn_points_path,motor_spawn_points_path,motor_pre_spawn_path,car_pre_spawn_path)
ped_spawn_points,ped_side_walk,ped_direction,ped_spawn_end_points = get_sidewalk_data(ped_spawns_points_path,ped_side_walk_path,ped_direction_path,ped_spawn_end_points_path)  

class env():
	def __init__(self,ped_num = 20,save_folder = 'datas',no_render_mode = False,ped_arrive_rate = 3000,veh_arrive_rate = 3000,total_frame = 3000):		
		
		self.world = world(ped_num,weather,save_folder,no_render_mode = no_render_mode)
		self.veh_ev = veh_env(self.world,save_folder,veh_arrive_rate,total_frame,self,vehicles_spawn_end_points,car_spawn_points,motor_spawn_points,graph_attr,motor_pre_spawn,car_pre_spawn)		
		self.graph = graph(self.world,self,graph_attr)
		self.ped_ev = ped_env(self.world,save_folder,ped_arrive_rate,total_frame,self,graph_attr,ped_spawn_point = ped_spawn_points,ped_side_walk = ped_side_walk,ped_direction = ped_direction,ped_spawn_end_points = ped_spawn_end_points)
		
		self.veh_ev.set_ped_env(self.ped_ev)
		self.veh_ev.set_graph(self.graph)
		
	def destroy(self):
		self.world.destroy()
		
vehicles = []		
v = None
simulate_time = 1
times_list = []

if save_folder == 'scenario_d':
	ped_arrive_rate = 100
elif save_folder == 'scenario_e':
	ped_arrive_rate = 100
elif save_folder == 'scenario_f':
	ped_arrive_rate = 1000
elif save_folder == 'scenario_g':
	ped_arrive_rate = 50

def main():	
	try:			
		for count in range(simulate_time):
		
			para_path = 'para_config/'+save_folder+'/parameter.csv'
			no_render_mode = False		
			veh_arrive_rate,total_frame,min_ped_speed,max_ped_speed,min_veh_desire_speed,max_veh_desire_speed,w1,w2,w3,w4,w5 = get_para_data(para_path)
			
			ped_datas,veh_datas = [],[]
			v = env(ped_arrive_rate = ped_arrive_rate,veh_arrive_rate = veh_arrive_rate,no_render_mode = no_render_mode,save_folder = save_folder)
			
			if save_folder == 'scenario_g':
				ped_idx = spawn_player_pedestrian(v.ped_ev,carla.Location(94,-70,1.0),2,carla.Location(500.32307434082031,-23.087177276611328,1.0))
				pygame.init()
				display = pygame.display.set_mode(
					(640,480),	#1280,720
					pygame.HWSURFACE | pygame.DOUBLEBUF)
				display.fill((0,0,0))
				control = Keyboard(v.ped_ev.pedestrians[ped_idx],v.world,v.ped_ev,v.veh_ev)
				clock = pygame.time.Clock()

			v.ped_ev.set_para(w1,w2,w3,w4,w5)
			v.ped_ev.speed_under = min_ped_speed
			v.ped_ev.speed_up = max_ped_speed	
			v.veh_ev.para_random[3] = (min_veh_desire_speed,max_veh_desire_speed)
			
			#v.world.spectator.set_transform(carla.Transform(carla.Location(-158,0,50),carla.Rotation(-90,90,0)))			
			start = time.process_time()
			
			while True:	
				#for k in range(1,12):
				#	if v.veh_ev.frame_count == 0:
				#		print(carla.Vector3D(x=55.949539, y=132.561295, z=0.000000)+k*8*carla.Vector3D(-0.999984,-0.005593,0.0))
				#	v.world.draw_point(carla.Vector3D(x=55.949539, y=132.561295, z=0.000000)+k*8*carla.Vector3D(-0.999984,-0.005593,0.0),1/29,carla.Color(255,0,0))
				#sys.stdout.write("\r%d" % (v.ped_ev.frame_count))				
				#sys.stdout.flush()
				#v.world.draw_point(carla.Location(1.5,62.966034,0.2),1/29,carla.Color(0,0,255))	
				#for item in v.graph.layer_mid_point:
				#	pa = item[0]
				#	pb = item[1]
				#	v.world.draw_point(pa,1/29,carla.Color(255,0,0)) 
				#	v.world.draw_point(pb,1/29,carla.Color(0,255,0)) 
				#	
				#for item in ped_side_walk:
				#	if item[1] == 1:
				#		draw_rect(v,item[0][0],item[0][2],item[0][3],item[0][1],carla.Color(255,0,0))
				#	else:
				#		draw_rect(v,item[0][0],item[0][2],item[0][3],item[0][1],carla.Color(255,255,0))
				#
				#for item in ped_spawn_points:
				#	if item[1] == 1:
				#		v.world.draw_point(item[0],1/29,carla.Color(255,0,0))	
				#	else:
				#		v.world.draw_point(item[0],1/29,carla.Color(255,255,0)) 
				#		
				#for spawn_end in vehicles_spawn_end_points:						
				#	draw_rect(v,spawn_end[0],spawn_end[2],spawn_end[1],spawn_end[3],carla.Color(0,255,0))
				
				if save_folder == 'scenario_b':
				
					v.veh_ev.para_random[0] = (3,4)
					v.veh_ev.para_random[1] = (3,4)
					v.veh_ev.para_random[2] = (3,4)
					
				if save_folder == 'scenario_d':
				
					v.veh_ev.para_random[0] = (3,4)
					v.veh_ev.para_random[1] = (3,4)
					v.veh_ev.para_random[2] = (3,4)
				
				if save_folder == 'scenario_e':	
				
					v.veh_ev.para_random[0] = (3,4)
					v.veh_ev.para_random[1] = (3,4)
					v.veh_ev.para_random[2] = (3,4)

					if v.veh_ev.frame_count == 0:
						bp = random.choice(v.world.blueprintsSameCar_1)
						spawn_trs = [carla.Location(-15,65.466034,1.0),-180,6,7]
						veh_idx = spawn_car(v.veh_ev,bp=bp,spawn_trs=spawn_trs,blocking_veh = True)
						if veh_idx != -1:
							v.veh_ev.blocking_veh_idx.append(veh_idx)
							v.veh_ev.lanechange_info.update({veh_idx:([8,9],9,1,-165,-180,v.graph.layer_mid_point[8][0]+v.graph.graph_vec*0.4,62.966034,'right')})
						
						
						bp = random.choice(v.world.blueprintsSameCar_4)
						spawn_trs = [carla.Location(8.884445,62.966034,1.0),-180,8,9]
						veh_idx = spawn_car(v.veh_ev,bp=bp,spawn_trs=spawn_trs,blocking_veh = True)
						if veh_idx != -1:
							v.veh_ev.blocking_veh_idx.append(veh_idx)
							v.veh_ev.lanechange_info.update({veh_idx:([6,7],9,-1,-195,-180,v.graph.layer_mid_point[7][0]-v.graph.graph_vec*0.4,62.966034,'left')})	
						

						bp = random.choice(v.world.blueprintsSameCar_2)
						spawn_trs = [carla.Location(10.884445,72.966034,1.0),0,0,1]
						veh_idx = spawn_car(v.veh_ev,bp=bp,spawn_trs=spawn_trs,blocking_veh = True)
						if veh_idx != -1:
							v.veh_ev.blocking_veh_idx.append(veh_idx)
							v.veh_ev.lanechange_info.update({veh_idx:([2,3],9,-1,-15,0,v.graph.layer_mid_point[2][0]-v.graph.graph_vec*0.4,62.966034,'left')})	
						
						bp = random.choice(v.world.blueprintsSameCar)
						spawn_trs = [carla.Location(-5.884445,72.966034,2.0),0,0,1]
						veh_idx = spawn_car(v.veh_ev,bp=bp,spawn_trs=spawn_trs,blocking_veh = True)

						if veh_idx != -1:
							v.veh_ev.blocking_veh_idx.append(veh_idx)
							v.veh_ev.lanechange_info.update({veh_idx:([2,3],9,-1,-15,0,v.graph.layer_mid_point[2][0]-v.graph.graph_vec*0.4,62.966034,'left')})	

						bp = random.choice(v.world.blueprintsSameCar_3)
						spawn_trs = [carla.Location(-15.884445,72.966034,2.0),0,0,1]
						veh_idx = spawn_car(v.veh_ev,bp=bp,spawn_trs=spawn_trs,blocking_veh = True)
						
						#print(v.veh_ev.vehicles[veh_idx].get_location() - v.veh_ev.veh_rear[veh_idx]*v.veh_ev.vehicles[veh_idx].get_transform().get_forward_vector())
						#carla.Location(-9.133330,65.466034,0.000000)
						#v.world.draw_point(carla.Location(5.133330,65.466034,0.000000),1/29,carla.Color(255,0,0)) 

						if veh_idx != -1:
							v.veh_ev.blocking_veh_idx.append(veh_idx)
							v.veh_ev.lanechange_info.update({veh_idx:([2,3],9,-1,-15,0,v.graph.layer_mid_point[2][0]-v.graph.graph_vec*0.4,62.966034,'left')})	
				
				if save_folder == 'scenario_f':	
				
					v.veh_ev.para_random[0] = (3,4)
					v.veh_ev.para_random[1] = (3,4)
					v.veh_ev.para_random[2] = (3,4)

					
				v.veh_ev.step()				   
				if save_folder != 'scenario_g':
					v.graph.update()				   
				v.ped_ev.step()
				
				if save_folder == 'scenario_g':
					clock.tick_busy_loop(60)
					if control.parse_events(clock):
						return 
				
				#command = v.veh_ev.veh_command + v.ped_ev.peds_command
				#v.world.client.apply_batch_sync(command,False)
				v.world.carla_world.tick()
				#v.veh_ev.veh_command.clear()
				#v.ped_ev.peds_command.clear()
							
				if v.ped_ev.frame_count >= total_frame:
					veh_datas.append(v.veh_ev.veh_data)
					ped_datas.append(v.ped_ev.ped_data)
					for agent in v.veh_ev.vehicles + v.ped_ev.pedestrians:
						if agent.is_alive:
							agent.destroy() 
					v.destroy()
					break		
					
			end = time.process_time()
			print("執行時間：%f 秒" % (end - start))
			times_list.append(end - start)	
			save_simulate_data(save_folder,ped_datas,veh_datas)
			
		#for t in times_list:
		#	print("執行時間：%f 秒" %t)	
		#header = [i + 1 for i in range(simulate_time)]
		#filepath = 'elapse_time/no_render_mode.csv'
		#write_csv(filepath,header,[times_list])
		
	except KeyboardInterrupt:
		pass
	finally:
		#kill_server()
		#os.system('shutdown -s')	
		for agent in v.veh_ev.vehicles + v.ped_ev.pedestrians:
			if agent.is_alive:
				agent.destroy() 
		v.destroy()

if __name__ == '__main__': 
	try:
		args = parse_args()
		main()
	except KeyboardInterrupt:
		pass			
	finally:
		print('end')
