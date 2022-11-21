import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
import matplotlib.pyplot as plt 
from tooluse import negative_vector,Alin,Asin,Aexp,fexp,flm,vecs2angle,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,isCollision,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
try:
	sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla
from numpy import random
#Driving force
def DrivingForce(ped_ev):
	pos = ped_ev.pos()
	vel = ped_ev.vel()
	goal = ped_ev.goal()

	direction, dist = stateutils.normalize(goal - pos)
	driving_force = np.zeros((ped_ev.size(), 2))
	#driving_force = (direction*ped_ev.max_speeds.reshape((-1, 1))-vel.reshape((-1, 2)))
	driving_force[dist > ped_ev.goal_threshold] = (direction *ped_ev.max_speeds.reshape((-1, 1))-vel.reshape((-1, 2)))[dist > ped_ev.goal_threshold, :]
	driving_force[dist <= ped_ev.goal_threshold] = -1.0 * vel[dist <= ped_ev.goal_threshold]
	driving_force /= ped_ev.tau
	return driving_force
#Obstacle force
def gateForce(ped_ev):
	pos_diff = ped_ev.gate_close	
	diff_direction, diff_length = stateutils.normalize(pos_diff)	
	diff_direction*=np.exp(-(diff_length.reshape(-1, 1)) / 1.0)
	vehiclesForce = diff_direction
	return vehiclesForce
def Left_region_boundry_force(ped_ev):
	pos_diff = ped_ev.left_boundry
	diff_direction, diff_length = stateutils.normalize(pos_diff)	
	diff_direction*=np.exp(-(diff_length.reshape(-1, 1)) / 1.0)
	Force = diff_direction
	return Force	
def Right_region_boundry_force(ped_ev):
	pos_diff = ped_ev.right_boundry
	diff_direction, diff_length = stateutils.normalize(pos_diff)	
	diff_direction*=np.exp(-(diff_length.reshape(-1, 1)) / 1.0)
	Force = diff_direction
	return Force	
def Rear_region_boundry_force(ped_ev):
	pos_diff = ped_ev.rear_boundry
	diff_direction, diff_length = stateutils.normalize(pos_diff)	
	diff_direction*=np.exp(-(diff_length.reshape(-1, 1)) / 1.0)
	Force = diff_direction
	return Force
#Social force
def SocialForce(ped_ev):	  
	pos_diff = stateutils.each_diff(ped_ev.pos())
	diff_direction, diff_length = stateutils.normalize(pos_diff)
	diff_direction*=np.exp(ped_ev.pedestrain_r*2-(diff_length.reshape(-1, 1)) / 1.0)
	social_force = np.sum(diff_direction.reshape((ped_ev.size(), -1, 2)), axis=1)
	return social_force	
def SocialForce2(ped_ev):
	lambda_importance = 2.0
	gamma = 0.35
	n = 2
	n_prime = 3
	
	pos_diff = stateutils.each_diff(ped_ev.pos())	 # n*(n-1)x2 other - ped_ev

	diff_direction, diff_length = stateutils.normalize(pos_diff)

	vel_diff = -1.0 * stateutils.each_diff(ped_ev.vel()) # n*(n-1)x2 ped_ev - other
	
	# compute interaction direction t_ij
	interaction_vec = lambda_importance * vel_diff + diff_direction
	interaction_direction, interaction_length = stateutils.normalize(interaction_vec)
	
	# compute angle theta (between interaction and position difference vector)
	theta = stateutils.vector_angles(interaction_direction) - stateutils.vector_angles(
		diff_direction
	)
	# compute model parameter B = gamma * ||D||
	B = gamma * interaction_length
	
	force_velocity_amount = np.exp(-1.0 * diff_length / B - np.square(n_prime * B * theta))
	force_angle_amount = -np.sign(theta) * np.exp(-1.0 * diff_length / B - np.square(n * B * theta))

	force_velocity = force_velocity_amount.reshape(-1, 1) * interaction_direction
	force_angle = force_angle_amount.reshape(-1, 1) * stateutils.left_normal(
		interaction_direction
	)
		
	force = force_velocity + force_angle	# n*(n-1) x 2
	force = np.sum(force.reshape((ped_ev.size(), -1, 2)), axis=1)
	
	
	return force