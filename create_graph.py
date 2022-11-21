import glob
import os
import sys
import time
import numpy as np
import math
import random
import stateutils
from tooluse import get_proj_len2,draw_graph,get_proj_len,isCollision,line_in_rect,get_Node_Width_Height,CellCollision,RectCollisionY,RectCollisionX,RectCollision,vector2list,vectorIsrightOf,vectorInfrontOf,cross,rotationVec,cross_point,isInRect,normalize,Length,dot,getMinMax,projectLengthOnto,getNorm,find_index_2,find_index,get_mid_point
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
from numpy import random

class graph:
	def __init__(self,world,updateL,graph_attr):
		self.veh_ev = updateL.veh_ev
		self.world = world
		
		self.graph_srcpoint_1 = graph_attr[0]
		self.graph_srcpoint_2 = graph_attr[1]
		self.graph_vec = graph_attr[2]
		self.layers_num = graph_attr[3]
		self.layers_height = graph_attr[4]
		self.graph_width_vec = graph_attr[5]
		self.graph_width = self.graph_srcpoint_1.distance(self.graph_srcpoint_2)
		self.graph_srcpoint = self.veh_ev.motor_spawn_points[0][0]-(self.layers_height/2)*self.graph_vec		
		self.each_layer_point_1,self.each_layer_point_2,self.layer_vector,self.layer_mid_point,self.layers_rect = self.get_each_layer_points()
		self.node_gap = [{} for k in range(self.layers_num)]
		self.node_mid_pair_points = [{} for k in range(self.layers_num)]
		self.region_to_layer = {}
		
		self.layers_node = []
		self.layers_region = []
		self.G = {}
		self.GT = {}	
		self._node_region_dict = {}
		self._region_node_dict = {}	
		
		for c,item in enumerate(self.layer_mid_point):
			pa = item[0]
			pb = item[1]
			print(c,':',pa,pb)
			
			
		for c,item in enumerate(self.layer_mid_point):
			if c < len(self.layer_mid_point) - 1:
				print(c,',',c+1,':',(self.layer_mid_point[c][0]+self.layer_mid_point[c+1][0])/2)
				print(c,',',c+1,':',(self.layer_mid_point[c][1]+self.layer_mid_point[c+1][1])/2)
	def get_each_layer_points(self):
		
		side_a = []
		lc = self.graph_srcpoint
		#self.world.draw_point(lc,1/29,carla.Color(255,0,0))
		for i in range(self.layers_num):
			lc2 = lc + self.layers_height*self.graph_vec
			side_a.append((lc,lc2))
			#self.world.draw_point(lc,1/29,carla.Color(255,0,0),size = 0.1)
			#self.world.draw_line(lc2,lc,1/29,carla.Color(0,255,0))
			lc = lc2
		#self.world.draw_point(lc,1/29,carla.Color(255,0,0))
			
		side_b = []
		lc = self.graph_srcpoint + self.graph_width*self.graph_width_vec
		#self.world.draw_point(lc,1/29,carla.Color(255,0,0))
		for i in range(self.layers_num):
			lc2 = lc + self.layers_height*self.graph_vec
			side_b.append((lc,lc2))
			#self.world.draw_point(lc,1/29,carla.Color(255,0,0),size = 0.1)
			#self.world.draw_line(lc2,lc,1/29,carla.Color(0,255,0))
			lc = lc2
		#self.world.draw_point(lc,1/29,carla.Color(255,0,0))
		layer_vector = []
		layer_mid_point = []
		for it1,it2 in zip(side_a,side_b):
			p1 = (it1[0] + it1[1])/2
			p2 = (it2[0] + it2[1])/2
			#self.world.draw_line(p1,p2,1/29,carla.Color(255,0,0))
			layer_vector.append(normalize(p1 - p2))
			layer_mid_point.append((p1,p2))
		
		layers_rect = []
		for i in range(self.layers_num):
			layers_rect.append((side_a[i][0],side_a[i][1],side_b[i][0],side_b[i][1]))
			
		#for it in side_a:
		#	self.world.draw_point(it[0],1/29,carla.Color(255,0,0))
		#	self.world.draw_point(it[1],1/29,carla.Color(255,0,0))
		#
		#for it in side_b:
		#	self.world.draw_point(it[0],1/29,carla.Color(255,0,0))
		#	self.world.draw_point(it[1],1/29,carla.Color(255,0,0))
		
		#self.world.draw_point(side_a[0][0],1/29,carla.Color(255,0,0))
		#self.world.draw_point(side_a[0][1],1/29,carla.Color(0,255,0))
		#
		#self.world.draw_point(side_b[0][0],1/29,carla.Color(255,0,0))
		#self.world.draw_point(side_b[0][1],1/29,carla.Color(0,255,0))
		
		return side_a,side_b,layer_vector,layer_mid_point,layers_rect		
	def draw_node(self,n,color = carla.Color(200,2,200)):
		p1,p2,p3,p4 = n[0],n[1],n[2],n[3]
		p1.z,p2.z,p3.z,p4.z = 0.2,0.2,0.2,0.2
		self.world.draw_line(p1,p3,(1/29),color)
		self.world.draw_line(p3,p4,(1/29),color)
		self.world.draw_line(p4,p2,(1/29),color)
		self.world.draw_line(p2,p1,(1/29),color)	 
	def update(self):		
		self.layers_node = []	
		self.G = {}
		self.GT = {}
		self.node_gap = [{} for k in range(self.layers_num)]
		self.node_mid_pair_points = [{} for k in range(self.layers_num)]
		
		#for rect in self.layers_rect:
		#	self.draw_node(rect)
			
		for layer_count,veh_array in enumerate(self.veh_ev.lanes_array):
			nodes = []
			veh_in = False
			for i,veh_idx in enumerate(veh_array):
				veh_in = True
				
				if i == 0:
					node = (-1,veh_idx)
					nodes.append(node)
					
					if dot(self.veh_ev.vehicles[node[1]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:
						p1 = self.veh_ev.vehicles[node[1]].get_location() + self.veh_ev.veh_rear[node[1]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]					
						d = get_proj_len2(p1,p4,p3)
						proj_p1 = p3 + d * normalize(p4 - p3)
						self.node_gap[layer_count].update({node:carla.Location(proj_p1.x,proj_p1.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(proj_p1,p4)})
						#self.world.draw_line(proj_p1,p4,(1/29),carla.Color(0,255,0))
					else:
						p1 = self.veh_ev.vehicles[node[1]].get_location() - self.veh_ev.veh_rear[node[1]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]	
						d = get_proj_len2(p1,p3,p4)
						proj_p2 = p4 + d * normalize(p3 - p4)
						self.node_gap[layer_count].update({node:carla.Location(proj_p2.x,proj_p2.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(proj_p2,p3)})
				if i > 0:
					node = (veh_array[i-1],veh_idx)
					nodes.append(node)
					
					if dot(self.veh_ev.vehicles[node[1]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:
						p1 = self.veh_ev.vehicles[node[0]].get_location() - self.veh_ev.veh_rear[node[0]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]					
						d = get_proj_len2(p1,p4,p3)
						proj_p1 = p3 + d * normalize(p4 - p3)

						p1 = self.veh_ev.vehicles[node[1]].get_location() + self.veh_ev.veh_rear[node[1]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]					
						d = get_proj_len2(p1,p4,p3)
						proj_p2 = p3 + d * normalize(p4 - p3)
						#self.world.draw_line(proj_p1,proj_p2,(1/29),carla.Color(255,0,0))
						self.node_gap[layer_count].update({node:carla.Location(proj_p2.x,proj_p2.y,0.0).distance(carla.Location(proj_p1.x,proj_p1.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(proj_p2,proj_p1)})
					else:
						p1 = self.veh_ev.vehicles[node[0]].get_location() + self.veh_ev.veh_rear[node[0]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]	
						d = get_proj_len2(p1,p3,p4)							   
						proj_p1 = p4 + d * normalize(p3 - p4)
						
						
						p1 = self.veh_ev.vehicles[node[1]].get_location() - self.veh_ev.veh_rear[node[1]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]	
						d = get_proj_len2(p1,p3,p4)
						proj_p2 = p4 + d * normalize(p3 - p4)
						self.node_gap[layer_count].update({node:carla.Location(proj_p2.x,proj_p2.y,0.0).distance(carla.Location(proj_p1.x,proj_p1.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(proj_p2,proj_p1)})
						#self.world.draw_line(proj_p1,proj_p2,(1/29),carla.Color(255,0,0))
					
				if i == len(veh_array)-1:
					node = (veh_idx,-1)
					nodes.append(node)	
					
					if dot(self.veh_ev.vehicles[node[0]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:
						p1 = self.veh_ev.vehicles[node[0]].get_location() - self.veh_ev.veh_rear[node[0]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]					
						d = get_proj_len2(p1,p4,p3)
						proj_p1 = p3 + d * normalize(p4 - p3)
						self.node_gap[layer_count].update({node:carla.Location(proj_p1.x,proj_p1.y,0.0).distance(carla.Location(p3.x,p3.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(p3,proj_p1)})
						#self.world.draw_line(proj_p1,p3,(1/29),carla.Color(0,0,255))
					else:
						p1 = self.veh_ev.vehicles[node[0]].get_location() + self.veh_ev.veh_rear[node[0]]*self.graph_width_vec
						p3 = self.layer_mid_point[layer_count][0]
						p4 = self.layer_mid_point[layer_count][1]	
						d = get_proj_len2(p1,p3,p4)							   
						proj_p1 = p4 + d * normalize(p3 - p4)
						self.node_gap[layer_count].update({node:carla.Location(proj_p1.x,proj_p1.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))})
						self.node_mid_pair_points[layer_count].update({node:(p4,proj_p1)})
			
			if not veh_in:
				node = (-1,-1)
				nodes.append(node)
				
				p3 = self.layer_mid_point[layer_count][0]
				p4 = self.layer_mid_point[layer_count][1]
				self.node_gap[layer_count].update({node:carla.Location(p3.x,p3.y,0.0).distance(carla.Location(p4.x,p4.y,0.0))})
				self.node_mid_pair_points[layer_count].update({node:(p3,p4)})
				
			self.layers_node.append(nodes)

			
		self.layers_region = []		
		self._node_region_dict = {}
		self._region_node_dict = {}

		#self.world.draw_point(self.each_layer_point_1[0][0],1/29,carla.Color(255,0,0))
		#self.world.draw_point(self.each_layer_point_1[len(self.each_layer_point_1)-1][1],1/29,carla.Color(0,255,0))
		#
		#self.world.draw_point(self.each_layer_point_2[0][0],1/29,carla.Color(255,0,0))
		#self.world.draw_point(self.each_layer_point_2[len(self.each_layer_point_2)-1][1],1/29,carla.Color(0,255,0))
		
		self.region_to_layer = {}
		
		for i,layer in enumerate(self.layers_node):
			regions = []
			for j,node in enumerate(layer):
				if node[0] == -1 and node[1] == -1:
					
					region = (self.each_layer_point_1[i][0],self.each_layer_point_1[i][1],self.each_layer_point_2[i][1],self.each_layer_point_2[i][0])
					self._node_region_dict.update({node:region})
					self._region_node_dict.update({region:node})
					
					self.region_to_layer.update({region:i})
					
					regions.append(region)
					
					
				elif node[0] == -1 and node[1] != -1:
					if self.veh_ev.vehicles[node[1]].is_alive:
						if dot(self.veh_ev.vehicles[node[1]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:	 
							p1 = self.veh_ev.vehicles[node[1]].get_location()+self.veh_ev.veh_rear[node[1]]*self.graph_width_vec 
							p3 = self.each_layer_point_2[i][1]
							p4 = self.each_layer_point_2[i][0]
							d = get_proj_len(p1,p4,p3)
							
							region = (p3,p4,p4-d*self.graph_width_vec,p3-d*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)
						else:
							p1 = self.veh_ev.vehicles[node[1]].get_location()-self.veh_ev.veh_rear[node[1]]*self.graph_width_vec 
							p3 = self.each_layer_point_1[i][1]
							p4 = self.each_layer_point_1[i][0]
							d = get_proj_len(p1,p4,p3)
							
							region =(p3,p4,p4+d*self.graph_width_vec,p3+d*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)
				elif node[0] != -1 and node[1] == -1: 
					if self.veh_ev.vehicles[node[0]].is_alive:
						if dot(self.veh_ev.vehicles[node[0]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:	 
							p1 = self.veh_ev.vehicles[node[0]].get_location()-self.veh_ev.veh_rear[node[0]]*self.graph_width_vec 
							p3 = self.each_layer_point_1[i][1]
							p4 = self.each_layer_point_1[i][0]
							d = get_proj_len(p1,p4,p3)
							
							region = (p3,p4,p4+d*self.graph_width_vec,p3+d*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)
						else:
							p1 = self.veh_ev.vehicles[node[0]].get_location()+self.veh_ev.veh_rear[node[0]]*self.graph_width_vec 
							p3 = self.each_layer_point_2[i][1]
							p4 = self.each_layer_point_2[i][0]
							d = get_proj_len(p1,p4,p3)
							
							region = (p3,p4,p4-d*self.graph_width_vec,p3-d*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)
				else:
					if self.veh_ev.vehicles[node[0]].is_alive and self.veh_ev.vehicles[node[1]].is_alive:
						if dot(self.veh_ev.vehicles[node[0]].get_transform().get_forward_vector(),self.graph_width_vec) > 0:
							
							p1 = self.veh_ev.vehicles[node[1]].get_location()+self.veh_ev.veh_rear[node[1]]*self.graph_width_vec 
							p3 = self.each_layer_point_1[i][1]
							p4 = self.each_layer_point_1[i][0]
							d1 = get_proj_len(p1,p4,p3)
							
  
							p5 = self.veh_ev.vehicles[node[0]].get_location()-self.veh_ev.veh_rear[node[0]]*self.graph_width_vec 
							p6 = self.each_layer_point_2[i][1]
							p8 = self.each_layer_point_2[i][0]
							d2 = get_proj_len(p5,p8,p6)
							
							region = (p3+d1*self.graph_width_vec,p4+d1*self.graph_width_vec,p8-d2*self.graph_width_vec,p6-d2*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)
						else:
							p1 = self.veh_ev.vehicles[node[1]].get_location()-self.veh_ev.veh_rear[node[1]]*self.graph_width_vec 
							p3 = self.each_layer_point_2[i][1]
							p4 = self.each_layer_point_2[i][0]
							d1 = get_proj_len(p1,p4,p3)
							
  
							p5 = self.veh_ev.vehicles[node[0]].get_location()+self.veh_ev.veh_rear[node[0]]*self.graph_width_vec 
							p6 = self.each_layer_point_1[i][1]
							p8 = self.each_layer_point_1[i][0]
							d2 = get_proj_len(p5,p8,p6)
							
							region = (p3-d1*self.graph_width_vec,p4-d1*self.graph_width_vec,p8+d2*self.graph_width_vec,p6+d2*self.graph_width_vec)
							self._node_region_dict.update({node:region})
							self._region_node_dict.update({region:node})
							
							self.region_to_layer.update({region:i})
							
							regions.append(region)

			self.layers_region.append(regions)
			
		#for i in range(len(self.layers_region)-1):
		#	for r_i in self.layers_region[i]:
		#	
		#		p1,p2,p3,p4 = r_i[0],r_i[1],r_i[2],r_i[3]
		#
		#		if not (p1,p2,p3,p4) in self.G:
		#			self.G.update({(p1,p2,p3,p4):[]})
		#		
		#		if not (p1,p2,p3,p4) in self.GT:
		#			self.GT.update({(p1,p2,p3,p4):[]})
		#			 
		#		for r_j in self.layers_region[i+1]:
		#		
		#			p5,p6,p7,p8 = r_j[0],r_j[1],r_j[2],r_j[3]
		#
		#			if not (p5,p6,p7,p8) in self.G:
		#				 self.G.update({(p5,p6,p7,p8):[]})
		#				 
		#			if not (p5,p6,p7,p8) in self.GT:
		#				 self.GT.update({(p5,p6,p7,p8):[]})
		#			
		#			r1_width = self.node_gap[i][self._region_node_dict[r_i]]
		#			r2_width = self.node_gap[i+1][self._region_node_dict[r_j]]
		#			
		#			if RectCollision(get_mid_point((p1,p2,p3,p4)),get_mid_point((p5,p6,p7,p8)),r1_width/2,self.layers_height/2,r2_width/2,self.layers_height/2):
		#				self.G[(p1,p2,p3,p4)].append((p5,p6,p7,p8))
		#				self.GT[(p5,p6,p7,p8)].append((p1,p2,p3,p4))
						
		for i in range(len(self.layers_region)-1):
		
			for r_i in self.layers_region[i]:
			
				p1,p2,p3,p4 = r_i[0],r_i[1],r_i[2],r_i[3]
			   
				maxX1,minX1 = max(p1.x,p2.x,p3.x,p4.x),min(p1.x,p2.x,p3.x,p4.x)
		
				if not (p1,p2,p3,p4) in self.G:
					self.G.update({(p1,p2,p3,p4):[]})
				
				if not (p1,p2,p3,p4) in self.GT:
					self.GT.update({(p1,p2,p3,p4):[]})
					 
				for r_j in self.layers_region[i+1]:
				
					p5,p6,p7,p8 = r_j[0],r_j[1],r_j[2],r_j[3]
					
					maxX2,minX2 = max(p5.x,p6.x,p7.x,p8.x),min(p5.x,p6.x,p7.x,p8.x)
		
					if not (p5,p6,p7,p8) in self.G:
						self.G.update({(p5,p6,p7,p8):[]})
						 
					if not (p5,p6,p7,p8) in self.GT:
						self.GT.update({(p5,p6,p7,p8):[]})
		
					if maxX1 >= minX2 and maxX2 >= minX1 and min(abs(maxX1 - minX1),abs(maxX2 - minX2),abs(maxX1 - minX2),abs(maxX2 - minX1)) > 0.6:
						self.G[(p1,p2,p3,p4)].append((p5,p6,p7,p8))
						self.GT[(p5,p6,p7,p8)].append((p1,p2,p3,p4))				
		
		draw_graph(self.G,self.world)
