import glob
import os
import sys
import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import random
import matplotlib.pyplot as plt 
from random import randrange
import logging
import re
from numpy import random
from tooluse import dot,isInRect,get_matrix,get_bounding_box_coords,get_car_rear
try:
	import queue
except ImportError:
	import Queue as queue
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla

def find_weather_presets():
	rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
	name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
	presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
	return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

SetAutopilot = carla.command.SetAutopilot
SpawnActor = carla.command.SpawnActor
FutureActor = carla.command.FutureActor
synchronous_master = False
sync = True

class world():
	def __init__(self,ped_num,weather,save_folder,draw_mode = False,simulate_mode = True,no_render_mode = False):
		####vehicle####
		self.draw_mode = draw_mode
		self.simulate_mode = simulate_mode
		self.client = carla.Client('127.0.0.1', 2000)
		self.client.set_timeout(10.0)
		self.carla_world = self.client.get_world() 
		self.carla_world_map = self.carla_world.get_map()
		self.traffic_manager = self.client.get_trafficmanager(8000)
		self.fixed_delta_t = (1/30)# if save_folder != 'scenario_b' else 1/10
		self.save_folder = save_folder
		self.spectator = self.carla_world.get_spectator()
		if sync:
			self.traffic_manager.set_synchronous_mode(True)
			settings = self.carla_world.get_settings()
			settings.synchronous_mode = True
			settings.no_rendering_mode = no_render_mode
			settings.fixed_delta_seconds = self.fixed_delta_t
			self.carla_world.apply_settings(settings)
		
		####vehicle####
		self.blueprintsVehicles = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.ford.mustang") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.volkswagen.t2") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.mercedes.sprinter") if int(x.get_attribute('number_of_wheels')) == 4] + [x for x in self.carla_world.get_blueprint_library().filter("vehicle.mercedes.coupe_2020") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.seat.leon") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.mercedes.coupe") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.diamondback.century") if int(x.get_attribute('number_of_wheels')) == 4]+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.tesla.model3") if int(x.get_attribute('number_of_wheels')) == 4] +[x for x in self.carla_world.get_blueprint_library().filter("vehicle.audi.tt") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsTrucks = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.carlamotors.firetruck") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsByclcles = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.diamondback.century") if int(x.get_attribute('number_of_wheels')) == 2]
		self.blueprintsMotorcycles = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.yamaha.yzf") if int(x.get_attribute('number_of_wheels')) == 2]#+[x for x in self.carla_world.get_blueprint_library().filter("vehicle.diamondback.century") if int(x.get_attribute('number_of_wheels')) == 2]
		self.blueprintsSameCar_1 = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.mercedes.coupe") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsSameCar_2 = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.ford.mustang") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsSameCar_3 = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.audi.tt") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsSameCar_4 = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.mercedes.coupe_2020") if int(x.get_attribute('number_of_wheels')) == 4]
		self.blueprintsSameCar = [x for x in self.carla_world.get_blueprint_library().filter("vehicle.tesla.model3") if int(x.get_attribute('number_of_wheels')) == 4]
		####pedestrian####
		self.blueprintsWalkers = self.carla_world.get_blueprint_library().filter('walker.pedestrian.*')
		#self.blueprintsWalkers = self.carla_world.get_blueprint_library().filter('walker.pedestrian.0018')
		
		self.veh_rear = []
		self.veh_raduis = []
		self.vehblueprint = []
		
		self.pedestrians = []
		self.vehicles = []		
		
		self.weather_preset = find_weather_presets()
		self.weather_dic = {}
		for item in self.weather_preset:
			self.weather_dic.update({item[1]:item[0]})
		self.carla_world.set_weather(self.weather_dic[weather])
		#print(find_weather_presets())
		
		if not simulate_mode:
		
			if save_folder == 'scenario_b':
				camera_transform = carla.Transform(carla.Location(-158,0,40),carla.Rotation(-90,90,0))
			elif save_folder == 'scenario_c':
				camera_transform = carla.Transform(carla.Location(15,-64.5,35),carla.Rotation(-90,90,0))
			elif save_folder == 'scenario_d':
				camera_transform = carla.Transform(carla.Location(15,-64.5,35),carla.Rotation(-90,90,0))	
			elif save_folder == 'scenario_e':
				camera_transform = carla.Transform(carla.Location(1.5,66.966034,30),carla.Rotation(-90,90,0))
			elif save_folder == 'scenario_f':
				camera_transform = carla.Transform(carla.Location(15.048273,138.457581,35),carla.Rotation(-90,90,0))	
			elif save_folder == 'scenario_g':
				camera_transform = carla.Transform(carla.Location(85,-56.5,30),carla.Rotation(-90,135,0))	
				
			self._queues = []
			self.sensor_actors = []
			attached_actor = self.carla_world.spawn_actor(
				random.choice(self.blueprintsSameCar),
				camera_transform)
			attached_actor.set_simulate_physics(False)
			

			bound_x = 0.5 + attached_actor.bounding_box.extent.x
			bound_y = 0.5 + attached_actor.bounding_box.extent.y
			bound_z = 0.5 + attached_actor.bounding_box.extent.z
			
			camera_bp = self.carla_world.get_blueprint_library().find('sensor.camera.rgb')
			if camera_bp.has_attribute('image_size_x'):
				camera_bp.set_attribute('image_size_x',str(1280))
			if camera_bp.has_attribute('image_size_y'):
				camera_bp.set_attribute('image_size_y',str(720))	
			Attachment = carla.AttachmentType
			camera_rgb = self.carla_world.spawn_actor(
				camera_bp,
				carla.Transform(carla.Location(x=+0.5*bound_x, y=+0.0*bound_y, z=1.3*bound_z)),
				attach_to=attached_actor,
				attachment_type=Attachment.Rigid)
			self.sensor_actors.append(camera_rgb)
			self.sensor_actors.append(attached_actor)
			self.sensors = [camera_rgb]
			def make_queue(register_event):
				q = queue.Queue()
				register_event(q.put)
				self._queues.append(q)
			for sensor in self.sensors:
				make_queue(sensor.listen)
				
	def tick(self, timeout):
		self.frame = self.carla_world.tick()
		data = [self._retrieve_data(q, timeout) for q in self._queues]
		assert all(x.frame == self.frame for x in data)
		return data
	def _retrieve_data(self, sensor_queue, timeout):
		while True:
			data = sensor_queue.get(timeout=timeout)
			if data.frame == self.frame:
				return data								
	def draw_arrow(self,p_1,p_2,delta_time):
		debug = self.carla_world.debug
		debug.draw_arrow(begin = p_1, end = p_2, thickness=0.1, arrow_size=0.1,color=carla.Color(255,0,0), life_time=delta_time)
	def draw_string(self,p,delta_time,text,color):
		debug = self.carla_world.debug
		debug.draw_string(location = p, text = text, draw_shadow=False, color=color, life_time=delta_time)
	def draw_point(self,p,delta_time,colora,size=0.1):
		debug = self.carla_world.debug
		debug.draw_point(location = p,size = size,color=colora, life_time=delta_time)
	def draw_line(self,p_1,p_2,delta_time,colora):
		debug = self.carla_world.debug
		debug.draw_line(begin = p_1,end =p_2,thickness = 0.1,color = colora, life_time=delta_time) 
	def destroy(self):		 
		settings = self.carla_world.get_settings()
		settings.synchronous_mode = False
		settings.fixed_delta_seconds = None
		self.carla_world.apply_settings(settings)
		
		if not self.simulate_mode:
			for actor in self.sensor_actors:
				if actor.is_alive:
					actor.destroy()
