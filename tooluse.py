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
from random import randrange
import argparse
import logging
from numpy import random
import json
from sklearn import preprocessing
import csv

def softmax(l,rever = False):
	sumL_e = 0
	for e in l:
		sumL_e += math.exp(e)
	if rever:
		return [1-(math.exp(e)/sumL_e) for e in l]
	else:
		return [(math.exp(e)/sumL_e) for e in l]
def summax(l,rever = False):
	sumL_e = 0
	for e in l:
		sumL_e += e
	if rever:
		return [1-(e/sumL_e) for e in l]
	else:
		return [(e/sumL_e) for e in l]
def sigmoid(v,rever=False):
	if rever:
		return 1 - (1/(1+math.exp(-v)))
	else:
		return 1/(1+math.exp(-v))
def Relu(v,rever=False):
	if rever:
		return 1 - min(max(0,v),1)
	else:
		return min(max(0,v),1)
def NormalizeData(data,rever=False):
	normalized = preprocessing.normalize([data])
	normalized = normalized[0]
	if rever:
		li = []
		for c,n in enumerate(normalized):
			li.append(1-n)
		return li
	else:		
		return	normalized
def get_mid_point(p):
	p1,p2,p3,p4 = p[0],p[1],p[2],p[3]
	return carla.Location((((p1+p2)/2 +(p3+p4)/2)/2).x,(((p1+p2)/2 +(p3+p4)/2)/2).y,0.0)
def find_index(arr, n, K):
 
	# Lower and upper bounds
	start = 0
	end = n - 1
 
	# Traverse the search space
	while start<= end:
 
		mid =(start + end)//2
 
		if arr[mid] == K:
			return mid
 
		elif arr[mid] < K:
			start = mid + 1
		else:
			end = mid-1
 
	# Return the insert position
	return end + 1
def find_index_2(arr, n, K):
 
	# Lower and upper bounds
	start = 0
	end = n - 1
 
	# Traverse the search space
	while start<= end:
 
		mid =(start + (end-start))//2
 
		if arr[mid] == K:
			return mid
 
		elif arr[mid] < K:
			start = mid + 1
		else:
			end = mid-1
 
	# Return the insert position
	return end + 1
def getNorm(v):
	norms = []
	for i in range(1,len(v)):
		p1 = v[i-1]
		p2 = v[i]
		n = (-(p2[1] - p1[1]),p2[0] - p1[0])
		norms.append(n)
	p1 = v[len(v)-1]
	p2 = v[0]
	n = (-(p2[1] - p1[1]),p2[0] - p1[0])
	norms.append(n)
	return norms
def projectLengthOnto(vertex,axis):
	dotProduct = vertex[0]*axis[0] + vertex[1]*axis[1]
	length = (axis[0] ** 2 + axis[1] ** 2) ** 0.5
	return dotProduct/max(length,0.1)
def getMinMax(vertices,axis):
	min_DotProduct = projectLengthOnto(vertices[0],axis)
	max_DotProduct = projectLengthOnto(vertices[0],axis)
	
	for i in range(1,len(vertices)):
		temp = projectLengthOnto(vertices[i],axis)
		if temp < min_DotProduct:
			min_DotProduct = temp
			min_index = i
		

		if temp > max_DotProduct:
			max_DotProduct = temp;
			max_index = i
			
	return (min_DotProduct,max_DotProduct)	
def isCollision(vertices_1,vertices_2):
	norms_boxA = getNorm(vertices_1)
	norms_boxB = getNorm(vertices_2)
	MinMax_PA = getMinMax(vertices_1, norms_boxA[0])
	MinMax_PB = getMinMax(vertices_2, norms_boxA[0])
	MinMax_QA = getMinMax(vertices_1, norms_boxA[1])
	MinMax_QB = getMinMax(vertices_2, norms_boxA[1])
	
	MinMax_RA = getMinMax(vertices_1, norms_boxB[0])
	MinMax_RB = getMinMax(vertices_2, norms_boxB[0])
	MinMax_SA = getMinMax(vertices_1, norms_boxB[1])
	MinMax_SB = getMinMax(vertices_2, norms_boxB[1])
	
	separate_P = MinMax_PB[0] > MinMax_PA[1] or MinMax_PA[0] > MinMax_PB[1]
				 
	separate_Q = MinMax_QB[0] > MinMax_QA[1] or MinMax_QA[0] > MinMax_QB[1]
				 
	separate_R = MinMax_RB[0] > MinMax_RA[1] or MinMax_RA[0] > MinMax_RB[1]
				 
	separate_S = MinMax_SB[0] > MinMax_SA[1] or MinMax_SA[0] > MinMax_SB[1]
	
	isSeparated = separate_P or separate_Q or separate_R or separate_S
	
	return (not isSeparated)
def dot(v1,v2):
	return	v1.x*v2.x + v1.y*v2.y + v1.z*v2.z	 
def Length(vec):	
	x = vec.x
	y = vec.y
	z = vec.z
	length = (x**2 + y**2 + z**2)**0.5
	return length
def normalize(vec):
	x = vec.x
	y = vec.y
	z = vec.z
	length = (x**2 + y**2 + z**2)**0.5
	
	if length != 0:
		vec.x /= length
		vec.y /= length
		vec.z /= length
	else:
		vec = carla.Vector3D(0,0,0)
	return vec
def isInRect(p1,p2,p3,p4,p):
	inRect = False
	v1 = p2 - p1
	v2 = p3 - p1
	v3 = p4 - p1
	v = p - p1
	
	if dot(v,v1) < 0:
		return inRect
		
	if dot(v,v2) < 0:
		return inRect
		
	if dot(v,v3) < 0:
		return inRect
	
	v1 = p1 - p2
	v2 = p3 - p2
	v3 = p4 - p2
	v = p - p2
	
	if dot(v,v1) < 0:
		return inRect
		
	if dot(v,v2) < 0:
		return inRect
		
	if dot(v,v3) < 0:
		return inRect
		
	v1 = p1 - p3
	v2 = p2 - p3
	v3 = p4 - p3
	v = p - p3
	
	if dot(v,v1) < 0:
		return inRect
		
	if dot(v,v2) < 0:
		return inRect
		
	if dot(v,v3) < 0:
		return inRect
		
		
	v1 = p1 - p4
	v2 = p2 - p4
	v3 = p3 - p4
	v = p - p4
		
	if dot(v,v1) < 0:
		return inRect
		
	if dot(v,v2) < 0:
		return inRect
		
	if dot(v,v3) < 0:
		return inRect
	
	inRect = True
	
	return inRect	
def cross_point(line1, line2):
	point_is_exist = False
	x = y = 0
	x1,y1,x2,y2 = line1 
	x3,y3,x4,y4 = line2

	if (x2 - x1) == 0:
		k1 = None
		b1 = 0
	else:
		k1 = (y2 - y1) * 1.0 / (x2 - x1)
		b1 = y1 * 1.0 - x1 * k1 * 1.0

	if (x4 - x3) == 0:
		k2 = None
		b2 = 0
	else:
		k2 = (y4 - y3) * 1.0 / (x4 - x3) 
		b2 = y3 * 1.0 - x3 * k2 * 1.0

	if k1 is None:
		if not k2 is None:
			x = x1
			y = k2 * x1 + b2
			point_is_exist = True
	elif k2 is None:
		x = x3
		y = k1 * x3 + b1
	elif not k2 == k1:
		x = (b2 - b1) * 1.0 / (k1 - k2)
		y = k1 * x * 1.0 + b1 * 1.0
		point_is_exist = True

	return point_is_exist,[x, y]
def line_in_rect(rt,rd,ld,lt,pa,pb):
	
	if isInRect(rt,rd,ld,lt,pa) or isInRect(rt,rd,ld,lt,pb):
		return True
	
	_,mp1 = cross_point([rt.x,rt.y,ld.x,ld.y],[pa.x,pa.y,pb.x,pb.y])
	_,mp2 = cross_point([rd.x,rd.y,lt.x,lt.y],[pa.x,pa.y,pb.x,pb.y])
	mp1,mp2 = carla.Location(mp1[0],mp1[1],0.0),carla.Location(mp2[0],mp2[1],0.0)
	
	if isInRect(rt,rd,ld,lt,mp1) or isInRect(rt,rd,ld,lt,mp2):
		return True
	else:
		return False
		
def rotationVec(vec,degree):
	x = vec.x
	y = vec.y
	
	v = np.array([[x],[y]],dtype=np.float)
	
	theta = np.radians(degree)
	c, s = np.cos(theta), np.sin(theta)
	R = np.array(((c, -s), (s, c)))
	
	result = R.dot(v)
	
	return carla.Vector3D(result[0][0],result[1][0],0)
def cross(a, b):
	c = [a[1]*b[2] - a[2]*b[1],
		 a[2]*b[0] - a[0]*b[2],
		 a[0]*b[1] - a[1]*b[0]]

	return c
def vectorInfrontOf(a,b,front_vec):
	if dot(normalize(a-b),front_vec) >= 0:
		return True
	else:
		return False
def vectorIsrightOf(a,b,right_vec):
	if dot(normalize(a-b),right_vec) > 0:
		return True
	else:
		return False
def vector2list(v):
	return([v.x,v.y,v.z])
def RectCollision(r1, r2,r1_width,r1_height,r2_width,r2_height):

	minX1 = r1.x - r1_width
	maxX1 = r1.x + r1_width 
	minY1 = r1.y - r1_height
	maxY1 = r1.y + r1_height 
	minX2 = r2.x - r2_width
	maxX2 = r2.x + r2_width 
	minY2 = r2.y - r2_height
	maxY2 = r2.y + r2_height

	if maxX1 >= minX2 and maxX2 >= minX1 and maxY1 >= minY2 and maxY2 >= minY1:
		return True;
	else:
		return False;
def RectCollisionX(r1, r2,r1_width,r1_height,r2_width,r2_height):

	minX1 = r1.x - r1_width
	maxX1 = r1.x + r1_width 
	minX2 = r2.x - r2_width
	maxX2 = r2.x + r2_width 

	if maxX1 >= minX2 and maxX2 >= minX1:
		return True
	else:
		return False
def RectCollisionY(r1, r2,r1_width,r1_height,r2_width,r2_height):

	minY1 = r1.y - r1_height
	maxY1 = r1.y + r1_height 
	minY2 = r2.y - r2_height
	maxY2 = r2.y + r2_height

	if maxY1 >= minY2 and maxY2 >= minY1:
		return True;
	else:
		return False;

def CellCollision(minX1,maxX1,minY1,maxY1,minX2,maxX2,minY2,maxY2):
	result = False
	result = True if maxX1 >= minX2 and maxX2 >= minX1 and maxY1 >= minY2 and maxY2 >= minY1 else False
	return result

def negative_vector(vector):
	return carla.Vector3D(-vector.x,-vector.y,-vector.z)

def get_matrix(transform):
	"""
	Creates matrix from carla transform.
	"""
	rotation = transform.rotation
	location = transform.location
	c_y = np.cos(np.radians(rotation.yaw))
	s_y = np.sin(np.radians(rotation.yaw))
	c_r = np.cos(np.radians(rotation.roll))
	s_r = np.sin(np.radians(rotation.roll))
	c_p = np.cos(np.radians(rotation.pitch))
	s_p = np.sin(np.radians(rotation.pitch))
	matrix = np.matrix(np.identity(4))
	matrix[0, 3] = location.x
	matrix[1, 3] = location.y
	matrix[2, 3] = location.z
	matrix[0, 0] = c_p * c_y
	matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
	matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
	matrix[1, 0] = s_y * c_p
	matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
	matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
	matrix[2, 0] = s_p
	matrix[2, 1] = -c_p * s_r
	matrix[2, 2] = c_p * c_r
	return matrix
def get_bounding_box_coords(vehicle):
	cords = np.zeros((8, 4))
	extent = vehicle.bounding_box.extent
	cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
	cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
	cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
	cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
	cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
	cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
	cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
	cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
	
	bb_transform = carla.Transform(vehicle.bounding_box.location)
	bb_vehicle_matrix = get_matrix(bb_transform)
	vehicle_world_matrix = get_matrix(vehicle.get_transform())
	bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
	cords_x_y_z = np.dot(bb_world_matrix, np.transpose(cords))[:3, :]

	return cords_x_y_z
def get_car_rear(veh):
	cords_x_y_z = get_bounding_box_coords(veh)
	boxCoord_1 = cords_x_y_z[:,0]#右上 (下)
	boxCoord_2 = cords_x_y_z[:,1]#右下 (下)
	boxCoord_3 = cords_x_y_z[:,2]#左下 (下)
	boxCoord_4 = cords_x_y_z[:,3]#左上 (下)
	forward_vector = veh.get_transform().get_forward_vector()
	forward_vector.z = 0
	right_vector = veh.get_transform().get_right_vector()
	right_vector.z = 0
	
	pos = veh.get_location()
	pos.z = 0
	
	a = carla.Location(float(boxCoord_1[0]),float(boxCoord_1[1]),0) 
	c = carla.Location(float(boxCoord_4[0]),float(boxCoord_4[1]),0) 
		
	b = carla.Location(float(boxCoord_2[0]),float(boxCoord_2[1]),0) 
	d = carla.Location(float(boxCoord_3[0]),float(boxCoord_3[1]),0) 
	
	dis = pos - (a+b)/2
	dis2 = pos - (b+d)/2
	#print((dis.x**2 + dis.y**2)**0.5)
	#print((dis2.x**2 + dis2.y**2)**0.5)
	
	if (dis.x**2 + dis.y**2)**0.5 > (dis2.x**2 + dis2.y**2)**0.5:
		return (dis.x**2 + dis.y**2)**0.5,(dis2.x**2 + dis2.y**2)**0.5	
	else:
		return (dis2.x**2 + dis2.y**2)**0.5,(dis.x**2 + dis.y**2)**0.5
def compute_right_near_veh_region(ped_ev,p1,p2,p3,p4,mid_point,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold):
	dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
	pa = [p1,p2,p3,p4][np.argmin(dis_L)]
	pb = pa + front_vec*Node_Height

	direct = right_vec if vectorIsrightOf(mid_point,pa,right_vec) else negative_vector(right_vec)
	if Node_Width > threshold:
		pc = pb + direct*threshold
		pd = pa + direct*threshold
	else:
		pa,pb,pc,pd = p1,p2,p3,p4
	
	h = abs(max(pa.y,pb.y,pc.y,pd.y) - min(pa.y,pb.y,pc.y,pd.y))
	w = abs(max(pa.x,pb.x,pc.x,pd.x) - min(pa.x,pb.x,pc.x,pd.x))
	mid_point = get_mid_point((pa,pb,pc,pd)) -(h/2)*front_vec
	l = mid_point-((w/2)-0.2)*right_vec
	r = mid_point+(w/2)*right_vec
	left_len = get_len(ped_ev,l,threshold,front_vec)
	right_len = get_len(ped_ev,r,threshold,front_vec)
	if left_len < right_len:
		return (l,l+left_len*front_vec,r+left_len*front_vec,r)
	else:
		return (l,l+right_len*front_vec,r+right_len*front_vec,r)
def compute_left_near_veh_region(ped_ev,p1,p2,p3,p4,mid_point,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold):
	dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
	pa = [p1,p2,p3,p4][np.argmin(dis_L)]
	pb = pa + front_vec*Node_Height

	direct = right_vec if vectorIsrightOf(mid_point,pa,right_vec) else negative_vector(right_vec)
	if Node_Width > threshold:
		pc = pb + direct*threshold
		pd = pa + direct*threshold
	else:
		pa,pb,pc,pd = p1,p2,p3,p4
	
	h = abs(max(pa.y,pb.y,pc.y,pd.y) - min(pa.y,pb.y,pc.y,pd.y))
	w = abs(max(pa.x,pb.x,pc.x,pd.x) - min(pa.x,pb.x,pc.x,pd.x))
	mid_point = get_mid_point((pa,pb,pc,pd)) -(h/2)*front_vec
	l = mid_point-(w/2)*right_vec
	r = mid_point+((w/2)-0.2)*right_vec
	left_len = get_len(ped_ev,l,threshold,front_vec)
	right_len = get_len(ped_ev,r,threshold,front_vec)
	if left_len < right_len:
		return (l,l+left_len*front_vec,r+left_len*front_vec,r)
	else:
		return (l,l+right_len*front_vec,r+right_len*front_vec,r)
def compute_near_veh_region(ped_ev,p1,p2,p3,p4,mid_point,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold):
	dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
	pa = [p1,p2,p3,p4][np.argmin(dis_L)]
	pb = pa + front_vec*Node_Height

	direct = right_vec if vectorIsrightOf(mid_point,pa,right_vec) else negative_vector(right_vec)
	if Node_Width > threshold:
		pc = pb + direct*threshold
		pd = pa + direct*threshold
	else:
		pa,pb,pc,pd = p1,p2,p3,p4
	
	h = abs(max(pa.y,pb.y,pc.y,pd.y) - min(pa.y,pb.y,pc.y,pd.y))
	w = abs(max(pa.x,pb.x,pc.x,pd.x) - min(pa.x,pb.x,pc.x,pd.x))
	mid_point = get_mid_point((pa,pb,pc,pd)) -(h/2)*front_vec
	l = mid_point-((w/2)-0.1)*right_vec
	r = mid_point+((w/2)-0.5)*right_vec
	left_len = get_len(ped_ev,l,threshold,front_vec)
	right_len = get_len(ped_ev,r,threshold,front_vec)
	if left_len < right_len:
		return (l,l+left_len*front_vec,r+left_len*front_vec,r)
	else:
		return (l,l+right_len*front_vec,r+right_len*front_vec,r)
	
def compute_far_veh_region(p1,p2,p3,p4,mid_point,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold):
	dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
	pa = [p1,p2,p3,p4][np.argmax(dis_L)]
	pb = pa - front_vec*Node_Height

	direct = right_vec if vectorIsrightOf(mid_point,pa,right_vec) else negative_vector(right_vec)
	if Node_Width > threshold:
		pc = pb + direct*threshold
		pd = pa + direct*threshold
	else:
		pa,pb,pc,pd = p1,p2,p3,p4
	return pa,pb,pc,pd
def get_len(ped_ev,p,ln,front_vec):
	label_points = ped_ev.world.carla_world.cast_ray(carla.Location(p.x,p.y,0.5),carla.Location(p.x,p.y,0.5)+front_vec*ln)
	casted_point = None
	for label_point in label_points:
		if label_point.label == carla.CityObjectLabel.Vehicles:
			casted_point = label_point.location
			break
	if casted_point != None:
		casted_point.z = 0.0
		return casted_point.distance(carla.Location(p.x,p.y,0.0))
	else:
		return ln
	
	
def get_h(ped_ev,front_region,front_vec,right_vec,G,ped_p,left_gridi,front_gridi,right_gridi,threshold):
	
	grids = [left_gridi,front_gridi,right_gridi]
	output_grids = []
	
	for grid in grids:
		if grid != None:
			h = abs(max(grid[0].y,grid[1].y,grid[2].y,grid[3].y) - min(grid[0].y,grid[1].y,grid[2].y,grid[3].y))
			w = abs(max(grid[0].x,grid[1].x,grid[2].x,grid[3].x) - min(grid[0].x,grid[1].x,grid[2].x,grid[3].x))
			mid_point = get_mid_point(grid) -(h/2)*front_vec
			dis2ped = abs(mid_point.y - ped_p.y)
			l = mid_point-(w/2)*right_vec-front_vec*dis2ped
			r = mid_point+(w/2)*right_vec-front_vec*dis2ped
			left_len = get_len(ped_ev,l,threshold,front_vec)
			right_len = get_len(ped_ev,r,threshold,front_vec)
			dis2ped = abs(l.y - ped_p.y)
			if left_len < right_len:
				if left_len >= (threshold/2):
					output_grids.append((l,l+left_len*front_vec,r+left_len*front_vec,r))
				else:	
					l = mid_point-(w/2)*right_vec
					r = mid_point+(w/2)*right_vec
					left_len = get_len(ped_ev,l,threshold,front_vec)
					right_len = get_len(ped_ev,r,threshold,front_vec)
					if left_len < right_len:
						output_grids.append((l,l+(left_len-dis2ped)*front_vec,r+(left_len-dis2ped)*front_vec,r))
					else:
						output_grids.append((l,l+(right_len-dis2ped)*front_vec,r+(right_len-dis2ped)*front_vec,r))
			else:
				if right_len >= (threshold/2):
					output_grids.append((l,l+right_len*front_vec,r+right_len*front_vec,r))
				else:	
					l = mid_point-(w/2)*right_vec
					r = mid_point+(w/2)*right_vec
					left_len = get_len(ped_ev,l,threshold,front_vec)
					right_len = get_len(ped_ev,r,threshold,front_vec)
					if left_len < right_len:
						output_grids.append((l,l+(left_len-dis2ped)*front_vec,r+(left_len-dis2ped)*front_vec,r))
					else:
						output_grids.append((l,l+(right_len-dis2ped)*front_vec,r+(right_len-dis2ped)*front_vec,r))
				#output_grids.append((l,l+right_len*front_vec,r+right_len*front_vec,r))
			#ped_ev.world.draw_line(l,l+left_len*front_vec,0.2,carla.Color(255,0,0))
			#ped_ev.world.draw_line(r,r+right_len*front_vec,0.2,carla.Color(255,0,0))
		else:
			output_grids.append(None)
	return output_grids[0],output_grids[1],output_grids[2]
			
	
def compute_front_veh_region(ped_ev,p1,p2,p3,p4,front_region,mid_point,ped_p,m,right_vec,front_vec,Node_Width,Node_Height,threshold,G):
	right_top = None
	left_top = None
	right_down = None
	left_down = None
	
	l = [p1,p2,p3,p4] 
	
	for p in l:
		if vectorIsrightOf(p,mid_point,right_vec) and vectorInfrontOf(p,mid_point,front_vec):
			right_top = p
		elif not vectorIsrightOf(p,mid_point,right_vec) and vectorInfrontOf(p,mid_point,front_vec):
			left_top = p
		elif vectorIsrightOf(p,mid_point,right_vec) and not vectorInfrontOf(p,mid_point,front_vec):
			right_down = p
		else:
			left_down = p
	
	
	left_down,right_down,left_top,right_top = left_down+right_vec*0.5+front_vec*0.5,right_down-right_vec*0.5+front_vec*0.5,left_top+right_vec*0.5-front_vec*0.5,right_top-right_vec*0.5-front_vec*0.5
	right_gridi = None
	front_gridi = None
	left_gridi = None
	
	#G[front_region]
	
	left_nearest = sys.maxsize
	right_nearest = sys.maxsize
	if Node_Width/2 > threshold:
		pa = right_top - right_vec*threshold
		pb = right_down - right_vec*threshold
		#pa = pb + front_vec*threshold
		pc = left_top + right_vec*threshold
		pd = left_down + right_vec*threshold
		#pc = pd + front_vec*threshold
		
		W = carla.Location(pa.x,pa.y,0.0).distance(carla.Location(pc.x,pc.y,0.0))
		mL,tmpRT,tmpRD = [],pa,pb
		numOfRegion = W//threshold
		remain_w = W%threshold
		
		for k in range(int(numOfRegion)):
			mL.append((tmpRT,tmpRD,tmpRD-right_vec*threshold,tmpRT-right_vec*threshold))
			tmpRT,tmpRD = tmpRT-right_vec*threshold,tmpRD-right_vec*threshold 
			if k == int(numOfRegion)-1:
				mL.append((tmpRT,tmpRD,tmpRD-right_vec*remain_w,tmpRT-right_vec*remain_w))
				

		lg,rg = (right_top,right_down,pb,pa),(pc,pd,left_down,left_top)
		#lg,rg = (right_down+ front_vec*threshold,right_down,pb,pa),(pc,pd,left_down,left_down+front_vec*threshold)
		mL.append(lg)
		mL.append(rg)
		for z in mL:
			maxX,minX = max(z[0].x,z[1].x,z[2].x,z[3].x),min(z[0].x,z[1].x,z[2].x,z[3].x)
			if m == (13,12):
				if maxX < ped_p.x and minX < ped_p.x:
					if right_nearest > abs(maxX - ped_p.x):
						right_nearest = abs(maxX - ped_p.x)
						right_gridi = z
				elif maxX > ped_p.x and minX > ped_p.x:
					if left_nearest > abs(minX - ped_p.x):
						left_nearest = abs(minX - ped_p.x)
						left_gridi = z
				else:
					front_gridi = z
			else:
				if maxX > ped_p.x and minX > ped_p.x:
					if right_nearest > abs(minX - ped_p.x):
						right_nearest = abs(minX - ped_p.x)
						right_gridi = z
				elif maxX < ped_p.x and minX < ped_p.x:
					if left_nearest > abs(maxX - ped_p.x):
						left_nearest = abs(maxX - ped_p.x)
						left_gridi = z
				else:
					front_gridi = z
		
		#if front_gridi != None:
		#	ped_ev.draw_node(front_gridi[0],front_gridi[1],front_gridi[2],front_gridi[3])
		#if left_gridi != None:
		#	ped_ev.draw_node(left_gridi[0],left_gridi[1],left_gridi[2],left_gridi[3])
		#if right_gridi != None:
		#	ped_ev.draw_node(right_gridi[0],right_gridi[1],right_gridi[2],right_gridi[3])
		
		#return (right_top,right_down,pb,pa),mL,(pc,pd,left_down,left_top)
	else:
		z = (p1,p2,p3,p4)
		maxX,minX = max(z[0].x,z[1].x,z[2].x,z[3].x),min(z[0].x,z[1].x,z[2].x,z[3].x)
		if m == (13,12):
			if maxX < ped_p.x and minX < ped_p.x:
				right_gridi = z
			elif maxX > ped_p.x and minX > ped_p.x:
				left_gridi = z
			else:
				front_gridi = z
		else:
			if maxX > ped_p.x and minX > ped_p.x:
				right_gridi = z
			elif maxX < ped_p.x and minX < ped_p.x:
				left_gridi = z
			else:
				front_gridi = z
		
		#if front_gridi != None:
		#	ped_ev.draw_node(front_gridi[0],front_gridi[1],front_gridi[2],front_gridi[3])
		#if left_gridi != None:
		#	ped_ev.draw_node(left_gridi[0],left_gridi[1],left_gridi[2],left_gridi[3])
		#if right_gridi != None:
		#	ped_ev.draw_node(right_gridi[0],right_gridi[1],right_gridi[2],right_gridi[3])		
			
		#if vectorIsrightOf(mid_point,ped_p,right_vec):
		#	return (p1,p2,p3,p4),[],None
		#else:
		#	return None,[],(p1,p2,p3,p4)
	left_gridi,front_gridi,right_gridi = get_h(ped_ev,front_region,front_vec,right_vec,G,ped_p,left_gridi,front_gridi,right_gridi,threshold)
	return left_gridi,front_gridi,right_gridi
	
def get_Grid(ped_ev,front_region,ped_p,m,right_vec,front_vec,threshold,G):
	left_gridi,front_gridi,right_gridi = None,None,None
	if front_region != None:
		p1,p2,p3,p4 = front_region[0],front_region[1],front_region[2],front_region[3]
		Node_Height = abs(max(p1.y,p2.y,p3.y,p4.y) - min(p1.y,p2.y,p3.y,p4.y))
		Node_Width = abs(max(p1.x,p2.x,p3.x,p4.x) - min(p1.x,p2.x,p3.x,p4.x))
		p1,p2,p3,p4,mid_point = front_region[0],front_region[1],front_region[2],front_region[3],get_mid_point(front_region)
		left_gridi,front_gridi,right_gridi = compute_front_veh_region(ped_ev,p1,p2,p3,p4,front_region,mid_point,ped_p,m,right_vec,front_vec,Node_Width,Node_Height,threshold,G)
	return left_gridi,front_gridi,right_gridi
	
def get_Left_Right_Front_Local_region(ped_ev,front_region,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold):
	
	p1,p2,p3,p4 = front_region[0],front_region[1],front_region[2],front_region[3]
	mid_point = get_mid_point(front_region)
	LR,MR,RR = compute_front_veh_region(p1,p2,p3,p4,mid_point,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold)
	
	left_Side,right_Side,left_Side_dis,right_Side_dis,nl,nr,front_grid = None,None,None,None,None,None,None

	front_grid = []
	not_front_grid = []
	bordering_grid = []
	if LR != None:	
		MR.append(LR)
		bordering_grid.append(LR)
	if RR != None:	   
		MR.append(RR)
		bordering_grid.append(RR)
		
	for m in MR:
		_,p = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[get_mid_point(m).x,get_mid_point(m).y,get_mid_point(m).x+right_vec.x,get_mid_point(m).y+right_vec.y])		   
		if isInRect(m[0],m[1],m[2],m[3],carla.Location(p[0],p[1],0.0)):
			front_grid.append(m)
		else:
			not_front_grid.append(m)
		
	left_Side = [m for m in not_front_grid if not vectorIsrightOf(get_mid_point(m),ped_p,right_vec)]
	right_Side = [m for m in not_front_grid if vectorIsrightOf(get_mid_point(m),ped_p,right_vec)]		 
	
	left_Side_dis = [carla.Location(get_mid_point(lr).x,get_mid_point(lr).y,0.0).distance(ped_p) for lr in left_Side]
	right_Side_dis = [carla.Location(get_mid_point(rr).x,get_mid_point(rr).y,0.0).distance(ped_p) for rr in right_Side]
	
	if len(left_Side) > 0:
		nl = np.argsort(left_Side_dis)#left_Side[np.argsort(left_Side_dis)[0]]
		#ped_ev.draw_node(nl[0],nl[1],nl[2],nl[3],carla.Color(0,255,0))
	if len(right_Side) > 0:
		nr = np.argsort(right_Side_dis)#right_Side[np.argsort(right_Side_dis)[0]]				
		#ped_ev.draw_node(nr[0],nr[1],nr[2],nr[3],carla.Color(255,0,0))
	#for m in right_Side:
	#	p1,p2,p3,p4 = m[0],m[1],m[2],m[3]
	#	ped_ev.draw_node(p1,p2,p3,p4,carla.Color(0,255,0))
	#for m in left_Side:
	#	p1,p2,p3,p4 = m[0],m[1],m[2],m[3]
	#	ped_ev.draw_node(p1,p2,p3,p4,carla.Color(255,0,0))	 
	return left_Side,right_Side,left_Side_dis,right_Side_dis,nl,nr,front_grid,bordering_grid

def get_LFR_grid(ped_ev,front_region,ped_p,right_vec,front_vec,threshold):
	L,F,R = [],[],[]
	bordering_grid = []
	if front_region != None:
		p1,p2,p3,p4 = front_region[0],front_region[1],front_region[2],front_region[3]
		Node_Height = abs(max(p1.y,p2.y,p3.y,p4.y) - min(p1.y,p2.y,p3.y,p4.y))
		Node_Width = abs(max(p1.x,p2.x,p3.x,p4.x) - min(p1.x,p2.x,p3.x,p4.x))
		left_Side,right_Side,left_Side_dis,right_Side_dis,nl,nr,front_grid,bordering_grid = get_Left_Right_Front_Local_region(ped_ev,front_region,ped_p,right_vec,front_vec,Node_Width,Node_Height,threshold)
		
		if front_grid != None:
			F = front_grid

		if left_Side != None and len(left_Side) > 0:
			L = [left_Side[nl[index]] for index,r in enumerate(left_Side)]

		if right_Side != None and len(right_Side) > 0:
			R = [right_Side[nr[index]] for index,r in enumerate(right_Side)]

	return L,F,R,bordering_grid
	
def get_regions_crowds_information(ped_ev,m,V_ped_idxs,region,sub_region = None):

	peds_in_region = V_ped_idxs[region]
	
	if sub_region != None:
		diff_dir_ped_id = [ped_id for ped_id in peds_in_region if isInRect(sub_region[0],sub_region[1],sub_region[2],sub_region[3],ped_ev.pedestrians[ped_id].get_location()) and (ped_ev.ped_goal_node_num[ped_id],ped_ev.ped_current_goal_node_num[ped_id]) != m]
		same_dir_ped_id = [ped_id for ped_id in peds_in_region if isInRect(sub_region[0],sub_region[1],sub_region[2],sub_region[3],ped_ev.pedestrians[ped_id].get_location()) and (ped_ev.ped_goal_node_num[ped_id],ped_ev.ped_current_goal_node_num[ped_id]) == m]
	else:
		diff_dir_ped_id = [ped_id for ped_id in peds_in_region if (ped_ev.ped_goal_node_num[ped_id],ped_ev.ped_current_goal_node_num[ped_id]) != m]
		same_dir_ped_id = [ped_id for ped_id in peds_in_region if (ped_ev.ped_goal_node_num[ped_id],ped_ev.ped_current_goal_node_num[ped_id]) == m]
	
	return diff_dir_ped_id,same_dir_ped_id

def grid_value(num_not_same_direction,num_same_direction,future_dir,w1,w2,w3,w4):
	if num_not_same_direction + num_same_direction == 0:
		same_dir_ratio = 0
		not_same_dir_ratio = 0
		empty_grid = 1
	else:
		same_dir_ratio = num_same_direction/(num_not_same_direction + num_same_direction)
		not_same_dir_ratio = (num_not_same_direction/(num_not_same_direction + num_same_direction))
		empty_grid = 0
	return w1*same_dir_ratio+w2*not_same_dir_ratio+w3*empty_grid+w4*future_dir
	
def Compute_grid_value(ped_ev,m,V_ped_idxs,front_region,future_target,ped_p,front_vec,right_vec,L,F,R,w1,w2,w3,w4,unsafe):
	L_grid_value,F_grid_value,R_grid_value = 0,0,0
	
	future_near = ''
	if future_target != None:
		if vectorIsrightOf(get_mid_point(future_target),ped_p,right_vec):
			future_near = 'right'
		else:
			future_near = 'left'
			
		
	
	if front_region != None:	
		ped_in_front_region = V_ped_idxs[front_region]
		
		if L != None or F != None or R != None:
		
			if L != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = L)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)
				future_dir = 0 if unsafe else (1 if future_near == 'left' else 0)
				L_grid_value = grid_value(num_not_same_direction,num_same_direction,future_dir,w1,w2,w3,w4)
				
			if F != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = F)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)
				future_dir = 0
				F_grid_value = grid_value(num_not_same_direction,num_same_direction,future_dir,w1,w2,w3,w4)
			
			if R != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = R)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)				
				future_dir = 0 if unsafe else (1 if future_near == 'right' else 0)
				R_grid_value = grid_value(num_not_same_direction,num_same_direction,future_dir,w1,w2,w3,w4)

	return L_grid_value,F_grid_value,R_grid_value

def grid_value2(num_not_same_direction,num_same_direction,future_dir,current_dir,w1,w2,w3,w4):
	if num_not_same_direction + num_same_direction == 0:
		same_dir_ratio = 0
		not_same_dir_ratio = 0
		empty_grid = 1
	else:
		same_dir_ratio = num_same_direction/(num_not_same_direction + num_same_direction)
		not_same_dir_ratio = (num_not_same_direction/(num_not_same_direction + num_same_direction))
		empty_grid = 0
	return w1*same_dir_ratio+w2*not_same_dir_ratio+w3*empty_grid+w4*future_dir+0.0*current_dir


def Compute_grid_value2(ped_ev,m,V_ped_idxs,front_region,future_target,next_target,ped_p,front_vec,right_vec,L,F,R,w1,w2,w3,w4,unsafe):
	L_grid_value,F_grid_value,R_grid_value = 0,0,0
	
	current_near = ''
	if next_target != None:
		if vectorIsrightOf(get_mid_point(next_target),ped_p,right_vec):
			current_near = 'right'
		else:
			current_near = 'left'
	
	
	
	future_near = ''
	if future_target != None:
		if vectorIsrightOf(get_mid_point(future_target),ped_p,right_vec):
			future_near = 'right'
		else:
			future_near = 'left'
			
		
	
	if front_region != None:	
		ped_in_front_region = V_ped_idxs[front_region]
		
		if L != None or F != None or R != None:
		
			if L != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = L)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)
				future_dir = 0 if unsafe else (1 if future_near == 'left' else 0)
				current_dir = 1 if current_near == 'left' else 0
				L_grid_value = grid_value2(num_not_same_direction,num_same_direction,future_dir,current_dir,w1,w2,w3,w4)
				
			if F != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = F)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)
				future_dir = 0.5
				current_dir = 0.5
				F_grid_value = grid_value2(num_not_same_direction,num_same_direction,future_dir,current_dir,w1,w2,w3,w4)
			
			if R != None:
				diff_dir_ped_id,same_dir_ped_id = get_regions_crowds_information(ped_ev,m,V_ped_idxs,front_region,sub_region = R)
				num_not_same_direction , num_same_direction = len(diff_dir_ped_id) , len(same_dir_ped_id)				
				future_dir = 0 if unsafe else (1 if future_near == 'right' else 0)
				current_dir = 1 if current_near == 'right' else 0
				R_grid_value = grid_value2(num_not_same_direction,num_same_direction,future_dir,current_dir,w1,w2,w3,w4)

	return L_grid_value,F_grid_value,R_grid_value

def regions_distance(ped_ev,next_nodes,ped_p,front_vec,right_vec):
	dis_to_n = []
	for n in next_nodes:
		mid_point = get_mid_point(n)
		p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
		p1,p2,p3,p4 = carla.Location(p1.x,p1.y,0.0),carla.Location(p2.x,p2.y,0.0),carla.Location(p3.x,p3.y,0.0),carla.Location(p4.x,p4.y,0.0)
		Node_Height = abs(max(p1.y,p2.y,p3.y,p4.y) - min(p1.y,p2.y,p3.y,p4.y))
		rolling = True
		_,p = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[mid_point.x,mid_point.y,mid_point.x+right_vec.x,mid_point.y+right_vec.y])	
		bP = carla.Location(p[0],p[1],0.0) - front_vec*(Node_Height/2)
		
		if isInRect(p1,p2,p3,p4,carla.Location(p[0],p[1],0.0)):
			rolling = False
			
		if rolling:
			dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
			p = [p1,p2,p3,p4][np.argmin(dis_L)]
			length = ped_p.distance(carla.Location(p.x,p.y,0.0))	
			#ped_ev.world.draw_line(ped_p,carla.Location(p.x,p.y,0.2),0.2,carla.Color(255,0,0))
		else:
			length = ped_p.distance(carla.Location(bP.x,bP.y,0.0))
			#ped_ev.world.draw_line(ped_p,carla.Location(bP.x,bP.y,0.2),0.2,carla.Color(255,0,0))
		dis_to_n.append(length)
	return dis_to_n
def get_front_region(ped_ev,ped_p,next_node,front_vec,right_vec):
	front_region = None
	for v in next_node:
		mid_point = get_mid_point(v)
		p1,p2,p3,p4 = v[0],v[1],v[2],v[3]
		_,p = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[mid_point.x,mid_point.y,mid_point.x+right_vec.x,mid_point.y+right_vec.y])	
		if isInRect(p1,p2,p3,p4,carla.Location(p[0],p[1],0.0)):
			front_region = v
			break
	return front_region
def check_front_move_around(ped_ev,ped_p,G,n,front_vec,right_vec,count,goal_side_walk_node,root_list):
	_next_ = G[n]
	front_region = get_front_region(ped_ev,ped_p,_next_,front_vec,right_vec)
	if front_region == None:
		#root_list.append(n)
		return root_list
	else:
		root_list.append(front_region)
		count += 1
		return check_front_move_around(ped_ev,ped_p,G,front_region,front_vec,right_vec,count,goal_side_walk_node,root_list)

def check_dead(ped_ev,G,n,front_vec,right_vec,count,goal_side_walk_node,root_list):
	_next_ = G[n]
	if len(_next_) == 1:
		root_list.append(n)
		return check_dead(ped_ev,G,_next_[0],front_vec,right_vec,count,goal_side_walk_node,root_list)
	elif len(_next_) > 1:
		return False
	else:
		if goal_side_walk_node == n:
			return False
		else:			
			#for r in root_list:
			#	ped_ev.draw_node(r)
			#ped_ev.draw_node(n)
			return True

	
def get_Node_region_Left_Right_mid(right_v,front_v,Nh,Nw,n):

	p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
	
	mid_point = get_mid_point(n)
	rf,rr = mid_point + (Nw/2)*right_v+(Nh/2)*front_v,mid_point + (Nw/2)*right_v-(Nh/2)*front_v
	lf,lr = mid_point - (Nw/2)*right_v+(Nh/2)*front_v,mid_point - (Nw/2)*right_v-(Nh/2)*front_v

	_,rrp = cross_point([mid_point.x,mid_point.y,mid_point.x+right_v.x,mid_point.y+right_v.y],[rf.x,rf.y,rr.x,rr.y])
	_,llp = cross_point([mid_point.x,mid_point.y,mid_point.x+right_v.x,mid_point.y+right_v.y],[lf.x,lf.y,lr.x,lr.y])
	
	rrp = carla.Location(rrp[0],rrp[1],0.0)
	llp = carla.Location(llp[0],llp[1],0.0)
	
	return rrp,llp,(rf,rr,lf,lr)
	
def get_Node_Width_Height(n):
	p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
	Node_Height = abs(max(p1.y,p2.y,p3.y,p4.y) - min(p1.y,p2.y,p3.y,p4.y))
	Node_Width = abs(max(p1.x,p2.x,p3.x,p4.x) - min(p1.x,p2.x,p3.x,p4.x))	
	return Node_Height,Node_Width
	
def all_path(ped_ev,destination_point,G,n,front_vec,right_vec,remain,count,root_list):
	
	_next_ = G[n]	
	dis = []
	privious_point = []
	des_point = []
	for v in _next_:
		mid_point = get_mid_point(v)
		p1,p2,p3,p4 = v[0],v[1],v[2],v[3]
		p1,p2,p3,p4 = carla.Location(p1.x,p1.y,0.0),carla.Location(p2.x,p2.y,0.0),carla.Location(p3.x,p3.y,0.0),carla.Location(p4.x,p4.y,0.0)
		Height = abs(max(p1.y,p2.y,p3.y,p4.y) - min(p1.y,p2.y,p3.y,p4.y))
		roll = True
		_,p = cross_point([destination_point.x,destination_point.y,destination_point.x+front_vec.x,destination_point.y+front_vec.y],[mid_point.x,mid_point.y,mid_point.x+right_vec.x,mid_point.y+right_vec.y])	
		if isInRect(p1,p2,p3,p4,carla.Location(p[0],p[1],0.0)):
			roll = False
		b = carla.Location(p[0],p[1],0.0) - front_vec*(Height/2)
		if roll:
			dis_L = [p1.distance(destination_point),p2.distance(destination_point),p3.distance(destination_point),p4.distance(destination_point)]
			p = [p1,p2,p3,p4][np.argmin(dis_L)]
			travel_length = destination_point.distance(carla.Location(p.x,p.y,0.0))
			#destination_point = carla.Location(p.x,p.y,0)
			#ped_ev.world.draw_line(carla.Location(p.x,p.y,0.2),carla.Location(destination_point.x,destination_point.y,0.2),0.06,carla.Color(0,0,255))
			des_point.append(p)
			#destination_point = carla.Location(p.x,p.y,0)
		else:
			travel_length = destination_point.distance(carla.Location(b.x,b.y,0.0))
			#ped_ev.world.draw_line(carla.Location(b.x,b.y,0.2),carla.Location(destination_point.x,destination_point.y,0.2),0.06,carla.Color(0,0,255))
			#destination_point = carla.Location(b.x,b.y,0)
			des_point.append(b)
		dis.append(travel_length)
	
	count+=1
	if len(dis) > 0:
		sortedList = np.argsort(dis)
		nearest_prep = des_point[sortedList[0]]
		nearest = _next_[sortedList[0]]
		root_list.append(nearest)
		#destination_point = carla.Location(nearest_prep.x,nearest_prep.y,0.0)
		#n = nearest
		if count < 5:
			all_path(ped_ev,carla.Location(nearest_prep.x,nearest_prep.y,0.0),G,nearest,front_vec,right_vec,remain,count,root_list)
		
		#ped_ev.draw_node(nearest)
		#
		#ped_ev.world.draw_line(nearest_prep,carla.Location(destination_point.x,destination_point.y,0.2),0.2,carla.Color(0,0,255))
		#ped_ev.world.draw_line(carla.Location(b.x,b.y,0.2),carla.Location(destination_point.x,destination_point.y,0.2),0.06,carla.Color(0,0,255))
		
	else:
		for r in root_list:
			ped_ev.draw_node(r)
		

	#if len(dis) > 1:
	#	sortedList = np.argsort(dis)
	#	nearest_prep = des_point[sortedList[0]]
	#	nearest = _next_[sortedList[0]]
	#	
	#	second_prep = des_point[sortedList[1]]
	#	second = _next_[sortedList[1]]
	#	
	#	#all_path(ped_ev,carla.Location(nearest_prep.x,nearest_prep.y,0.0),G,nearest,front_vec,right_vec,remain)
	#	#all_path(ped_ev,carla.Location(second_prep.x,second_prep.y,0.0),G,second,front_vec,right_vec,remain)
	#	
	#	ped_ev.draw_node(nearest)
	#	ped_ev.draw_node(second)
	#	
	#	ped_ev.world.draw_line(nearest_prep,carla.Location(destination_point.x,destination_point.y,0.2),0.2,carla.Color(0,0,255))
	#	ped_ev.world.draw_line(second_prep,carla.Location(destination_point.x,destination_point.y,0.2),0.2,carla.Color(0,0,255))
	#	
	#elif len(dis) == 1:
	#	sortedList = np.argsort(dis)
	#	nearest_prep = des_point[sortedList[0]]
	#	nearest = _next_[sortedList[0]]
	#	
	#	#all_path(ped_ev,carla.Location(nearest_prep.x,nearest_prep.y,0.0),G,nearest,front_vec,right_vec,remain)
	#	
	#	ped_ev.draw_node(nearest)
	#	
	#	ped_ev.world.draw_line(nearest_prep,carla.Location(destination_point.x,destination_point.y,0.2),0.2,carla.Color(0,0,255))
	#	ped_ev.world.draw_line(carla.Location(b.x,b.y,0.2),carla.Location(destination_point.x,destination_point.y,0.2),0.06,carla.Color(0,0,255))
	#	
	#else:
	#	ped_ev.draw_node(n)

	#return all_path(ped_ev,destination_point,G,v,front_vec,right_vec)
def Alin(pi,lamda):
	return max(1-lamda*(np.fabs(pi)/np.pi),0)
def Asin(pi,lamda):
	return lamda + (1-lamda)*((1+np.cos(np.fabs(pi)))/2)
def Aexp(pi,lamda):
	return np.exp(-lamda*np.fabs(pi))
def fexp(d,A=1,B=1):
	return A*np.exp(-(B*d)/0.8)
def flm(d,d0,M,ro):
	return (M/(2*d0))*(d0-d+((d0-d)**2+ro)**0.5)
def get_next_grid(gridR,gridV):
	next_grid = None
	max_grid_value = -1
	max_grid_value_index = -1
	if len(gridV) > 0:
		max_grid_value_index = np.argmax(gridV)
		max_grid_value,next_grid = gridV[max_grid_value_index],gridR[max_grid_value_index]
	return next_grid,max_grid_value,max_grid_value_index
def draw_circles(ped_ev,mid,r,interpol = 0.1,point_size = 0.04 , z = 0.2,color = carla.Color(0,255,0)):
	max_theta = 2* np.pi
	list_t = list(np.arange(0,max_theta,interpol))
	circle_points = [(mid.x + r*math.cos(x_y),mid.y + r*math.sin(x_y)) for x_y in list_t]

	for p in circle_points:
		ped_ev.world.draw_point(carla.Location(p[0],p[1],z),1/29,color,size = point_size)
def nearest_next_nodes(ped_ev,next_nodes,front_vec,right_vec,ped_p,i):
	lengths = []
	roll = []
	for n in next_nodes:
		mid_point = get_mid_point(n)
		p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
		p1,p2,p3,p4 = carla.Location(p1.x,p1.y,0.0),carla.Location(p2.x,p2.y,0.0),carla.Location(p3.x,p3.y,0.0),carla.Location(p4.x,p4.y,0.0)
		_region_node_dict = ped_ev.graph._region_node_dict
		node = _region_node_dict[n]
		w = 0
		for dic_layer in ped_ev.graph.node_gap:
			if node in dic_layer:
				w = dic_layer[node]
				break	
		h = ped_ev.graph.layers_height
		
		rolling = True
		_,p = cross_point([ped_p.x,ped_p.y,ped_p.x+front_vec.x,ped_p.y+front_vec.y],[mid_point.x,mid_point.y,mid_point.x+right_vec.x,mid_point.y+right_vec.y])	
		if isInRect(p1,p2,p3,p4,carla.Location(p[0],p[1],0.0)):
			rolling = False
			
		bP = carla.Location(p[0],p[1],0.0) - front_vec*(h/2)

		if rolling:
			dis_L = [p1.distance(ped_p),p2.distance(ped_p),p3.distance(ped_p),p4.distance(ped_p)]
			p = [p1,p2,p3,p4][np.argmin(dis_L)]
			length = ped_p.distance(carla.Location(p.x,p.y,0.0))
		else:
			length = ped_p.distance(carla.Location(bP.x,bP.y,0.0))
		lengths.append(length)
		roll.append(rolling)
	
	if len(lengths) == 1:
		return next_nodes[np.argmin(lengths)],None
	elif len(lengths) > 1:
		return next_nodes[np.argmin(lengths)],next_nodes[np.argsort(lengths)[1]]
	else:
		return None,None
def get_proj_len(p1,p2,p3):
	va,vb = p1 - p3, p2 - p3
	dpc = abs(dot(va,vb))/math.sqrt((vb.x**2 + vb.y**2))
	d = math.sqrt(abs((va.x**2 + va.y**2) - (dpc ** 2)))
	return d
def get_proj_len2(p1,p2,p3):
	va,vb = p1 - p3, p2 - p3
	dpc = abs(dot(va,vb))/math.sqrt((vb.x**2 + vb.y**2))
	return dpc
	

def get_vec_proj_len(va,vb):
	dpc = abs(dot(va,vb))/math.sqrt((vb.x**2 + vb.y**2))
	return dpc	  
	


def draw_graph(G,world):
	for key in G:
		p1,p2,p3,p4 = key[0],key[1],key[2],key[3]
		for j in range(len(G[key])):
			p5,p6,p7,p8 = G[key][j][0],G[key][j][1],G[key][j][2],G[key][j][3]
			
			world.draw_line(p1,p2,(1/29),carla.Color(0,0,255))
			world.draw_line(p2,p3,(1/29),carla.Color(0,0,255))
			world.draw_line(p3,p4,(1/29),carla.Color(0,0,255))
			world.draw_line(p4,p1,(1/29),carla.Color(0,0,255))
			
			#world.draw_line(((p1+p2)/2 + (p3+p4)/2)/2,((p5+p6)/2 + (p7+p8)/2)/2,(1/29),carla.Color(255,0,0))
																				
			world.draw_line(p5,p6,(1/29),carla.Color(0,0,255))					
			world.draw_line(p6,p7,(1/29),carla.Color(0,0,255))					
			world.draw_line(p7,p8,(1/29),carla.Color(0,0,255))
			world.draw_line(p8,p5,(1/29),carla.Color(0,0,255))
def vecs2angle(v1,v2):
	x=np.array([v1.x,v1.y])
	y=np.array([v2.x,v2.y])
	Lx=np.sqrt(x.dot(x))
	Ly=np.sqrt(y.dot(y))
	cos_angle=x.dot(y)/(Lx*Ly)
	angle=np.arccos(cos_angle)
	return np.degrees(angle)
