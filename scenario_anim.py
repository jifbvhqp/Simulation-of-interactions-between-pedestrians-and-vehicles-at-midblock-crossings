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
from scenarioplayback import replay_animation,read_ped_csv
def read_csv(filepath):
	temp = []
	with open(filepath, newline='') as csvfile:
		rows = csv.reader(csvfile)
		for count,row in enumerate(rows):
			if count > 0:		  
				temp.append(row)
	return temp	   
def get_all_folder(path):
	folder_path = []
	allFileList = os.listdir(path)
	for count,file in enumerate(allFileList):		
		if os.path.isdir(os.path.join(path,file)):
			folder_path.append(file)
	return folder_path
def get_all_csv(path):
	file_path = []
	allFileList = os.listdir(path)
	for count,file in enumerate(allFileList):		
		if os.path.isdir(os.path.join(path,file)):
			pass
		else:
			file_path.append(file)
	return file_path
def get_para_data(para_path):
	
	basic_para_rows = read_csv(para_path)[0]
	scenario = ''
	scenario = basic_para_rows[0]
		
	return scenario
def get_saveimage_para(para_path):	
	para_rows = read_csv(para_path)[0]
	no_render_mode = False
	save_image = False
	no_render_mode = False if para_rows[0] == 'False' else True
	save_image = False if para_rows[1] == 'False' else True
	return no_render_mode,save_image

path = read_csv('scenario.csv')[0][0]
weather = 'Clear Noon'

save_image_path = 'save_image_para/parameter.csv'	
no_render_mode,save_image = get_saveimage_para(save_image_path)

all_anim_file_path = []
veh_csvs = []
ped_csvs = []
region_csvs = []

for folder_name in get_all_folder(path):
	for para_file in get_all_csv(path+'/'+folder_name):
		if para_file[-7:] == 'veh.csv':
			veh_csvs.append(path+'/'+folder_name+'/'+para_file)
		elif para_file[-10:] == 'region.csv':
			region_csvs.append(path+'/'+folder_name+'/'+para_file)
		elif para_file[-7:] == 'ped.csv':
			ped_csvs.append(path+'/'+folder_name+'/'+para_file)

replay_animation(ped_csvs,veh_csvs,path,weather,no_render_mode=no_render_mode,save_image = save_image)
