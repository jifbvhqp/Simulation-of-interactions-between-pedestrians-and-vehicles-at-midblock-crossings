from typing import Tuple
import glob
import os
import sys
import time
import numpy as np
import math
import re
import weakref
from scipy.spatial.transform import Rotation as R
import random
import stateutils
import tkinter as tk
import matplotlib.pyplot as plt 
from PIL import Image
from random import randrange
import argparse
import logging
from numpy import random
from carla_world import world
from veh_update import veh_env
import threading
import time
try:
	import pygame
	from pygame.locals import KMOD_CTRL
	from pygame.locals import KMOD_SHIFT
	from pygame.locals import K_0
	from pygame.locals import K_9
	from pygame.locals import K_BACKQUOTE
	from pygame.locals import K_BACKSPACE
	from pygame.locals import K_COMMA
	from pygame.locals import K_DOWN
	from pygame.locals import K_ESCAPE
	from pygame.locals import K_F1
	from pygame.locals import K_LEFT
	from pygame.locals import K_PERIOD
	from pygame.locals import K_RIGHT
	from pygame.locals import K_SLASH
	from pygame.locals import K_SPACE
	from pygame.locals import K_TAB
	from pygame.locals import K_UP
	from pygame.locals import K_a
	from pygame.locals import K_b
	from pygame.locals import K_c
	from pygame.locals import K_d
	from pygame.locals import K_g
	from pygame.locals import K_h
	from pygame.locals import K_i
	from pygame.locals import K_l
	from pygame.locals import K_m
	from pygame.locals import K_n
	from pygame.locals import K_p
	from pygame.locals import K_q
	from pygame.locals import K_r
	from pygame.locals import K_s
	from pygame.locals import K_v
	from pygame.locals import K_w
	from pygame.locals import K_y
	from pygame.locals import K_t
	from pygame.locals import K_x
	from pygame.locals import K_z
	from pygame.locals import K_u
	from pygame.locals import K_j
	from pygame.locals import K_k
	from pygame.locals import K_o
	from pygame.locals import K_1
	from pygame.locals import K_2
	from pygame.locals import K_3
	from pygame.locals import K_4
	from pygame.locals import K_5
	from pygame.locals import K_MINUS
	from pygame.locals import K_EQUALS
except ImportError:
	raise RuntimeError('cannot import pygame, make sure pygame package is installed')
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from carla import ColorConverter as cc
from carla import VehicleLightState as vls
from ped_update import ped_env
from veh_update import veh_env
import pygame


class Keyboard(object):
	def __init__(self,player,world,ped_env,veh_env):
		self.player = player
		if isinstance(self.player, carla.Walker):
			self._control = carla.WalkerControl()
			self._rotation = carla.Rotation(0,135,0)#self.player.get_transform().rotation
		elif isinstance(self.player, carla.Vehicle):
			self._control = carla.VehicleControl()
		self._steer_cache = 0.0
		self.world = world
		self.ped_env = ped_env
		self.veh_env = veh_env
	def parse_events(self, clock): 
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return True
			elif event.type == pygame.KEYUP:
				if self._is_quit_shortcut(event.key):
					return True
				if isinstance(self._control, carla.VehicleControl):
					if event.key == K_q:
						self._control.gear = 1 if self._control.reverse else -1
					
		if isinstance(self._control, carla.VehicleControl):
			self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
			self._control.reverse = self._control.gear < 0
		elif isinstance(self._control, carla.WalkerControl):
			self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
	
		self.player.apply_control(self._control)
	def _parse_walker_keys(self, keys, milliseconds):
		self._control.speed = 0.0
		if keys[K_DOWN] or keys[K_j]:
			self._control.speed = 0.0
		if keys[K_LEFT] or keys[K_h]:
			self._control.speed = .01
			self._rotation.yaw -= 0.08 * milliseconds
		if keys[K_RIGHT] or keys[K_k]:
			self._control.speed = .01
			self._rotation.yaw += 0.08 * milliseconds
		if keys[K_UP] or keys[K_u]:
			self._control.speed = 1.5
		self._control.jump = keys[K_SPACE]
		self._rotation.yaw = round(self._rotation.yaw, 1)
		self._control.direction = self._rotation.get_forward_vector()
	def _parse_vehicle_keys(self, keys, milliseconds):
		if keys[K_UP] or keys[K_u]:
			self._control.throttle = min(self._control.throttle + 0.01, 0.5)
		else:
			self._control.throttle = 0.0

		if keys[K_DOWN] or keys[K_j]:
			self._control.brake = min(self._control.brake + 0.2, 0.5)
		else:
			self._control.brake = 0

		steer_increment = 5e-4 * milliseconds
		if keys[K_LEFT] or keys[K_h]:
			if self._steer_cache > 0:
				self._steer_cache = 0
			else:
				self._steer_cache -= steer_increment
		elif keys[K_RIGHT] or keys[K_k]:
			if self._steer_cache < 0:
				self._steer_cache = 0
			else:
				self._steer_cache += steer_increment
		else:
			self._steer_cache = 0.0
		self._steer_cache = min(0.5, max(-0.5, self._steer_cache))
		self._control.steer = round(self._steer_cache, 1)
		self._control.hand_brake = keys[K_SPACE]	
	@staticmethod
	def _is_quit_shortcut(key):
		return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)