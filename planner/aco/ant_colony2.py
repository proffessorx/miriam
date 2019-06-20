#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: cch-student : Ali Qizilbash

source : https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms#Edge_selection
ACO __init__ : https://en.wikipedia.org/wiki/Ant_colony_optimization_algorithms
cumulative probability behavior : http://stackoverflow.com/a/3679747/5343977
zero increment : http://stackoverflow.com/a/10426033/5343977

Threading : 
Threading in Python : http://stackoverflow.com/a/11968818/5343977
"""

from threading import Thread
import time
import numpy as np
import math
from matplotlib import pyplot as plt
from planner.common import path
from tools import load_map
import networkx as nx

map = load_map('o.png')
#print (map)
grid = np.repeat(map[:, ::2, np.newaxis], 100, axis=2)
#print ("Grid::: ", grid)
#print grid.shape

map = map[:, ::2]
#print map
n = map.shape[0]
#print map.shape
G = nx.grid_graph([n, n])
#print ("G::: ", G)

obstacle = []
for n in G.nodes():
    if not map[n] >= 0:  # obstacle
        obstacle.append(n)

G.remove_nodes_from(obstacle)

# =============================================================================
# MAIN CLASS :::
# =============================================================================

class ant_colony:
	class ant(Thread):
		def __init__(self, init_location, pickup_locations, rob, pheromone_map, distance_callback, alpha, beta, first_pass=False):
			"""
            Initializes an ANT to traverse the map.
            
            init_location : Start point of ANT
            pickup_locations : LIST of Pickup Tasks visitable (unvisited)
            rob : Which Robot the ANST belong to
            pheromone_map : map of Ph Values
            distance_callback : distance between two Nodes
            
            alpha : Ph constant      (0.1 -- 0.9)
            beta : distance constant (0.1 -- 5.0)
            
            route :           LIST with labelled nodes already visited
            pheromone_trail : LIST of Ph values along the path (maps to each traversal in route)
            
            dist_traveled : total distance traveled in route
            location : current location of ANT
            
            first_pass :    boolean/flag for First tour
            tour_complete : boolean/flag when ANT has completed its traversal
            
			"""
			#for THREADING
			#Thread.__init__(self)
			
			self.init_location = init_location
			self.pickup_locations = pickup_locations		
			self.rob = rob
			self.route = []
			self.path = []
			self.dist_traveled = 0.0
			self.location = init_location
			self.pheromone_map = pheromone_map
			self.distance_callback = distance_callback
			self.alpha = alpha
			self.beta = beta
			self.first_pass = first_pass
			
			#append start location to route, before doing random walk
			self.route_update(init_location)
			
			self.tour_complete = False
			
# =============================================================================
# Ant Functions :::
# =============================================================================

		def run(self, robot, assigned):
			"""
            Traverse until all Possible locations are visited.
            robot: which ROBOT the ANT Belongs to
            
            pick_path() : Selecting Next Node to traverse
            traverse() : Move to next location
            alpha : Ph constant      (0.1 -- 0.9)
            beta : distance constant (0.1 -- 5.0)
			"""		
			#print ("RUN", assigned)
			if len(assigned) is not 0:
				for x in assigned:
					#print x
					self.pickup_locations.remove(x)
					#print ("removed &&&", assigned)

			#print ("Pickup :::", self.pickup_locations)    
			while self.pickup_locations:                
				next = self.pick_path(robot)
				#print ("This is  ", next)
				self.traverse(self.location, next, robot)
				
			self.tour_complete = True
			            
		def pick_path(self, robot):
            #TODO
			"""
			Path Selection in ACO
            
			calculate the attractiveness of each possible transition from the current location
			then randomly choose a next path, based on its attractiveness
            
			"""
			#on the first pass (no pheromones), then we can just choice() to find the next one
			if self.first_pass:
				import random
				return random.choice(self.pickup_locations)
			
			attractiveness = dict()
			sum_total = 0.0
			#print ("pickup_locations: ", robot, self.pickup_locations)
			#for each possible location, find its attractiveness (it's (pheromone amount)*1/distance [tau*eta, from the algortihm])
			#sum all attrativeness amounts for calculating probability of each route in the next step
			for possible_next_location in self.pickup_locations:
				#NOTE: do all calculations as float, otherwise we get integer division at times for really hard to track down bugs
				#print ("here: ", self.pheromone_map[robot][self.location][possible_next_location])
				pheromone_amount = float(self.pheromone_map[robot][self.location][possible_next_location])
				dist, pp = self.distance_callback(self.location, possible_next_location, robot)
				distance = float(dist)
				
				#tau^alpha * eta^beta
				attractiveness[possible_next_location] = pow(pheromone_amount, self.alpha)*pow(1/distance, self.beta)
				sum_total += attractiveness[possible_next_location]
			
			#it is possible to have small values for pheromone amount / distance, such that with rounding errors this is equal to zero
			#rare, but handle when it happens
			if sum_total == 0.0:
				#increment all zero's, such that they are the smallest non-zero values supported by the system

				def next_up(x):
					import math
					import struct
					# NaNs and positive infinity map to themselves.
					if math.isnan(x) or (math.isinf(x) and x > 0):
						return x

					# 0.0 and -0.0 both map to the smallest +ve float.
					if x == 0.0:
						x = 0.0

					n = struct.unpack('<q', struct.pack('<d', x))[0]
					
					if n >= 0:
						n += 1
					else:
						n -= 1
					return struct.unpack('<d', struct.pack('<q', n))[0]
					
				for key in attractiveness:
					attractiveness[key] = next_up(attractiveness[key])
				sum_total = next_up(sum_total)
			
			#randomly choose the next path
			import random
			toss = random.random()
					
			cummulative = 0
			for possible_next_location in attractiveness:
				weight = (attractiveness[possible_next_location] / sum_total)
				if toss <= weight + cummulative:
					return possible_next_location
				cummulative += weight
		
		def traverse(self, start, end, robot):
			"""
            Traversing or moving from node to node
            
			route_update() : route updating to new end point
			dist_traveled_update() updating new distance and adding distance from start point to end point
			Location update to New Location
            
			return the ants route and its distance, for use in ant_colony:
				do pheromone updates
				check for new possible optimal solution with this ants latest tour
            
			"""
			#print ("traverse ", start, end)            
			self.route_update(end)
			self.dist_traveled_update(start, end, robot)
			self.location = end
		
		def route_update(self, new):
			"""
            Adding new Node to Route and removing from Possible locations to be visited
            
			--- traverse()  and __init__() ---
			"""
			self.route.append(new)           
			#print ("removed", new)
			self.pickup_locations.remove(new)
			#print ("after removing ", self.pickup_locations)            

		def get_route(self):
			"""
            Returns Route if tour completed
            """
			if self.tour_complete:
				return self.route, self.path
			return None
			
		def dist_traveled_update(self, start, end, robot):
			"""
            Updates Distance Traveled by using distance_callback
			"""
			#print (start)
			#print (end)
            #tuple___coming
			dist, pp = self.distance_callback(start, end, robot)
			#print dist
			#print pp
			self.dist_traveled += float(dist)
			self.path.append(pp)            
				
		def get_dist_traveled(self):
			"""
            Returns Distance Traveled if tour completed
            """
			if self.tour_complete:
				return self.dist_traveled
			return None

# =============================================================================
# INITIALIZATION METHOD:::
# =============================================================================
		
	def __init__(self, nodes, distance_callback, start = None, robots = 1, ant_count=3, alpha=0.5,
              beta=1.2,  pheromone_evaporation_coefficient=.4, pheromone_constant=1000, iterations=1):
        
		"""
        Initializing an ANT Colony. 
		(ants will traverse the map to find optimal solution using pheromone and distances)
        "ants : holds worker ants as they traverse the map, properties: total distance traveled & route"
        ant_count : Number of ANTS generated
		
        robots : Number of Robots
            
		nodes : DICT, list with Labels to be used by distance_callback
			
		distance_callback : Gives/returns distance between > is assumed to take a pair of coordinates and return the distance between them
			populated into distance_matrix on each call to get_distance()
			
		(old) start : starting point of Ants. Default: start from first sorted key of Node
        start : should be starting Node of First Robot (Later, Multi-Robots) e.g. start = [(1,3), None]
		
		distance_matrix : matrix filled with Distances by get_distance()
        
        path_matrix :
		
		pheromone_map : final values of Pheromone
		pheromone dissipation happens to these values first,
        before adding pheromone values from the ants during their traversal
		(in ant_updated_pheromone_map)
			
		ant_updated_pheromone_map : matrix filled with Ph values laid by ANTS
		not used to dissipate, values from here are added to pheromone_map after dissipation step
		(reset for each traversal)
		
        alpha : Pheromone  constant to control influence of Ph while selecting Path          (0.1 -- 0.9)
        beta  : distance   constant to control influence of Distance while selecting Path    (1.0 -- 5.0)
        
		pheromone_constant                : "Q"   depositing pheromone factor used in ph_map_update()
		pheromone_evaporation_coefficient : "rho" evaporating pheromone factor used in ph_map_update()
		iterations : number of Iterations for ANTS to tranverse the Map	
		"""

		def check(multi_nodes):
			#create internal mapping and mapping for return to caller
			self.id_to_key, robot_nodes = self.nodes_init(multi_nodes)
			#print ("here ", robot_nodes[1])
			#self.id_to_key.append(id_key)
			self.multi_robot_nodes.append(robot_nodes)
			#create matrix to hold distance calculations between nodes
			#self.distance_matrix = self.matrix_init(len(multi_robot_nodes))
			self.distance_matrix.append(self.matrix_init(len(multi_nodes)))
			#print(self.distance_matrix)
    	    #create matrix to hold paths generated between nodes
			#self.path_matrix = self.matrix_init(len(multi_robot_nodes))
			self.path_matrix.append(self.matrix_init(len(multi_nodes)))
			#create matrix for master pheromone map, that records pheromone amounts along routes
			#self.pheromone_map = self.matrix_init(len(multi_robot_nodes))
			self.pheromone_map.append(self.matrix_init(len(multi_nodes)))
			#create a matrix for ants to add their pheromones to, before adding those to pheromone_map during the update_pheromone_map step
			#self.ant_updated_pheromone_map = self.matrix_init(len(multi_robot_nodes))
			self.ant_updated_pheromone_map.append(self.matrix_init(len(multi_nodes)))
			
			#distance_callback CHECK
			if not callable(distance_callback):
				raise TypeError("distance_callback is not callable, should be method")
			self.distance_callback = distance_callback
            
			#ant_count CHECK
			if type(ant_count) is not int:
				raise TypeError("ant_count must be int")
			if ant_count < 1:
				raise ValueError("ant_count must be >= 1")
			self.ant_count = ant_count
			
			#alpha CHECK	
			if (type(alpha) is not int) and type(alpha) is not float:
				raise TypeError("alpha must be int or float")
			if alpha < 0:
				raise ValueError("alpha must be >= 0")
			self.alpha = float(alpha)
			
			#beta CHECK
			if (type(beta) is not int) and type(beta) is not float:
				raise TypeError("beta must be int or float")
			if beta < 1:
				raise ValueError("beta must be >= 1")
			self.beta = float(beta)
			
			#pheromone_evaporation_coefficient CHECK
			if (type(pheromone_evaporation_coefficient) is not int) and type(pheromone_evaporation_coefficient) is not float:
				raise TypeError("pheromone_evaporation_coefficient must be int or float")
			self.pheromone_evaporation_coefficient = float(pheromone_evaporation_coefficient)
			
			#pheromone_constant CHECK
			if (type(pheromone_constant) is not int) and type(pheromone_constant) is not float:
				raise TypeError("pheromone_constant must be int or float")
			self.pheromone_constant = float(pheromone_constant)
			
			#iterations CHECK
			if (type(iterations) is not int):
				raise TypeError("iterations must be int")
			if iterations < 0:
				raise ValueError("iterations must be >= 0")
			self.iterations = iterations
			
			#other internal variable initialize 
			self.first_pass = True
			#print ("start: ", self.start)
			self.ants = self.ants_init(self.start)
			self.shortest_distance = None
			self.best_route_seen = None
			self.best_path_seen = None

		#nodes
		if type(nodes) is not dict:
			raise TypeError("nodes must be dict")
		
		if len(nodes) < 1:
			raise ValueError("there must be at least one node in dict nodes")
		#print(nodes)

		self.distance_matrix = []
		self.path_matrix = []
		self.pheromone_map = []
		self.ant_updated_pheromone_map = []
		self.multi_robot_nodes = []
		#self.id_to_key = []
        
        #start
		robot_nodes = []
		if start is None:
			self.start = 0
			robot_nodes = nodes            
		else:
			if start[1] is None:    
				''' When ONE starting Node/Robot is Given'''
				self.start = None
				#print ("Starting Point(0) and Tasks(1,2,..)")
				nodes = { i+1 : nodes[i] for i in range(0, len(nodes) ) }
				nodes.update( {0: start} )
				robot_nodes.append(nodes)
				self.robots = len(robot_nodes)
				self.start = 0
				check(robot_nodes[0])
			else: #TODO
				self.start = None
				robot_nodes = []
				temp = {}       
				for st in start:
					temp = { i+1 : nodes[i] for i in range(0, len(nodes) ) }
					temp.update( {0: st} )
					robot_nodes.append(temp)
				self.robots = 0
				self.start = 0                
				for i in robot_nodes:
					check(i)
					#instead of assigning directly to len(robot_nodes) as useful for loop counter
					self.robots+=1 
				                
# =============================================================================
# Main Functions :::
# =============================================================================
	
	def rdp(self, robot_assign):
		D = 0
		pp = []#print len(robot_assign)
		for r in range(len(robot_assign)):
			st = self.multi_robot_nodes[r][0]
			for e in range(len(robot_assign[r])):                
				en = self.multi_robot_nodes[r][robot_assign[r][e]] #robot_assign[0][0]           
				#print st, en, r
				temp_d, temp_pp = self.distance_callback(st, en)
				D += temp_d
				pp.append(temp_pp)
				#print D
		return D, pp
		
        
	def get_distance(self, start, end, robot):
		#print("get_distance")
		"""
		Returns Distance between Nodes.
        uses distance_callback
		if a distance not calculated before, then it is populated in distance_matrix and returned
		if a distance called before, then its value is returned from distance_matrix
		"""
		#print(self.distance_matrix[i][start][end])
		if not self.distance_matrix[robot][start][end]:
			#print ("here " , self.multi_robot_nodes[start])
			dist, pp = self.distance_callback(self.multi_robot_nodes[robot][start], self.multi_robot_nodes[robot][end])
			#print ("start " , self.multi_robot_nodes[start])
			if (type(dist) is not int) and (type(dist) is not float):
				raise TypeError("distance_callback should return either int or float, saw: "+ str(type(distance)))
			
			self.distance_matrix[robot][start][end] = float(dist)
			self.path_matrix[robot][start][end] = pp
			return dist, pp
		return self.distance_matrix[robot][start][end], self.path_matrix[robot][start][end]
		
	def nodes_init(self, nodes, id = 0):
		#print("nodes_init and ID assigning")
		#print(nodes)
		"""
		Initializing Nodes and assigning ID's 
		create a mapping of the id's to the values of nodes
		use id_to_key to return route in the node names the caller expects in main()
		"""
		id_to_key = dict()
		id_to_values = dict()
		
		for key in sorted(nodes.keys()):
			id_to_key[id] = key
			id_to_values[id] = nodes[key]
			id += 1
			
		return id_to_key, id_to_values
		
	def matrix_init(self, size, value=0.0):
		#print ("matrix_init")
		"""
		Initializing a Matrix  NxN (where n = size)
		for, self.distance_matrix and self.pheromone_map
		Also, Initializing with any Value possible
		"""
		ret = []
		for row in range(size):
			ret.append([float(value) for x in range(size)])
		return ret
	
	def ants_init(self, start, robot = None):
		#print ("ants_init")
		"""
		Initializing ANTS
        first_pass also new ants
        on Normal passes, just call __init__ on each ANT to reset them
        #Iteration number of times
		"""
		#allocate new ants on the first pass
		#print ("Robots ", self.robots)
        #TODO Here I have to give Assigned Tasks to Robots
#		temp = self.multi_robot_nodes[self.robots-1].keys()
#		if assignments is not None:
#			print ("assignments  ", self.multi_robot_nodes[self.robots-1].keys())
#			if type(assignments) is int:
#				temp.remove(assignments)
#			else:
#				for x in assignments:
#					#print("removing ", x)
#					temp.remove(x)                
#			print ("for the robot: ", robot, "new multi_robot_keys  ", temp)
            
		if self.first_pass:
			return [self.ant(start, self.multi_robot_nodes[self.robots].keys(), self.robots, self.pheromone_map, self.get_distance,
				self.alpha, self.beta, first_pass=True) for x in range(self.ant_count)]
		#else, just reset them to use on another pass
		for ant in self.ants:
			#print ("RESET:::::::::::::::::::::::::::: ")
			ant.__init__(start, self.multi_robot_nodes[self.robots-1].keys(), self.robots, self.pheromone_map, self.get_distance, self.alpha, self.beta)
	
	def ph_map_update(self, robot):
		#print("ph_map_update")
		"""
        Ph Map update by Decay and new Ph values deposited by ANTS
		called by main() after all ants are traversed.
        #Iterations number of times
		"""
		#always a square matrix
		for start in range(len(self.pheromone_map[robot])):
			for end in range(len(self.pheromone_map[robot])):
				#decay the pheromone value at this location
				#tau_xy <- (1-rho)*tau_xy	(ACO)
				self.pheromone_map[robot][start][end] = (1-self.pheromone_evaporation_coefficient)*self.pheromone_map[robot][start][end]
				
				#then add all contributions to this location for each ant that travered it
				#tau_xy <- tau_xy + delta tau_xy_k
				#	delta tau_xy_k = Q / L_k
				self.pheromone_map[robot][start][end] += self.ant_updated_pheromone_map[robot][start][end]
	
	def update_ant_updated_pheromone_map(self, ant, robot):
		#print("populate")
		"""
		For every ANT, updates new Ph Values to the ant_updated_pheromone_map
		along the ant's route
		called by main() before ph_map_update
        #ANT * Iterations number of times
		"""
		route, p = ant.get_route()
		for i in range(len(route)-1):
			#find the pheromone over the route the ant traversed
			current_pheromone_value = float(self.ant_updated_pheromone_map[robot][route[i]][route[i+1]])

			#update the pheromone along that section of the route
			#(ACO)
			#	delta tau_xy_k = Q / L_k
			new_pheromone_value = self.pheromone_constant/ant.get_dist_traveled()

			self.ant_updated_pheromone_map[robot][route[i]][route[i+1]] = current_pheromone_value + new_pheromone_value
			self.ant_updated_pheromone_map[robot][route[i+1]][route[i]] = current_pheromone_value + new_pheromone_value

	def update_pickup_locations(self, robot):
		#print ("update")
		"""

		"""
		_, tasks = self.nodes_init(test_nodes, 1)
		temp = [0] * self.robots
		sec_temp = [0] * self.robots        
		#print ("Map ", self.pheromone_map)
		for r in range(self.robots):
			#print ("robot: ", r)
			for st in range(len(self.pheromone_map[r][0])):
				for en in range(len(self.pheromone_map[r][0])):
					if temp[r] < self.pheromone_map[r][st][en]:
						temp[r] = r, st, en, self.pheromone_map[r][st][en]						
					if (temp[r] is not 0) and (temp[r][3] is not self.pheromone_map[r][st][en]) and (sec_temp[r] < self.pheromone_map[r][st][en]) and temp[r][1] is not en:
						sec_temp[r] = r, st, en, self.pheromone_map[r][st][en]
		#print ("Update : ", temp[robot], "2nd. ", sec_temp[robot])
#		if robot is 2:    
		#print ("Update : ", temp[robot])
		return temp[robot], sec_temp[robot]        
		#TODO Here I have to Assign Tasks to Robots
		#print ("here", robot,  tasks[4])
		#print ("ROBOT : ", tasks, robot,  self.ant_updated_pheromone_map[robot])

	def soft_assign(self, assign, robot_assign, robot):
		#print ("SOFT ASSIGN")
		"""
		 
		"""
		first_assign, sec_assign = self.update_pickup_locations(robot)
		if first_assign[2] in assign and sec_assign[2] in assign:
			print "yes"
		elif  first_assign[2] in assign and sec_assign[2] not in assign:
			first_assign = sec_assign
			assign.append(first_assign[2])
			#print first_assign[2]
			robot_assign[robot].append(assign[-1])
		elif first_assign[3] > sec_assign[3]/2 :
			assign.append(first_assign[2])
			#print first_assign[2]
			robot_assign[robot].append(assign[-1])
		elif assign is []:
			assign = None
		#robot_assign[robot].append(assign[robot])
        #assign[robot] = first_assign[2]                
		#print ("assign ", assign)
		#robot_assign[robot].append(assign[robot]) 		

            
# =============================================================================
# MAIN ALGO:::
# =============================================================================
		
	def main(self):
		#print("main")
		"""
        Runs Iteration Times
        Runs Threading : ant.start() and ant.join() 
		Runs ANTS, collects their returns and updates pheromone map
		"""
		dists = []
		route = []
		robot_assign = [[], [], []]
        #ITERATION#############################################################
		for _ in range(self.iterations):         
			assign = []
			s = 0                
            #WHILE#############################################################
			while s < (len(self.multi_robot_nodes[0])-1):
				#print ("robot_assign ", robot_assign)
				#print "WHILE with s: ", s, "len is: ", (len(self.multi_robot_nodes[0])-1)
            	#ROBOT#############################################################
				for robot in range(self.robots):                
					print "Robot:", robot
            	 	   
           		 #----------------THREADING----------------------    
					#start the multi-threaded ants, calls ant.run() in a new thread
#					for ant in self.ants:
#					ant.start()
#					
#					#wait until the ants are finished, before moving on to modifying shared resources
#					for ant in self.ants:
#						ant.join()
           		 #----------------THREADING----------------------  
            	    
            	    
					############################ANTS###############################
					for ant in self.ants:	
						ant.run(robot, assign)
						#update ant_updated_pheromone_map with this ant's constribution of pheromones along its route
						self.update_ant_updated_pheromone_map(ant, robot)
						
						#if we haven't seen any paths yet, then populate for comparisons later
						if not self.shortest_distance:
							self.shortest_distance = ant.get_dist_traveled()
						
						if not self.best_route_seen:
							self.best_route_seen, self.best_path_seen = ant.get_route()
						dists.append(self.shortest_distance)
            	  	  
						#if we see a shorter path, then save for return
						if ant.get_dist_traveled() < self.shortest_distance:
							self.shortest_distance = ant.get_dist_traveled()
							print ("Shortest Distance is %s " % self.shortest_distance )
							self.best_route_seen, self.best_path_seen = ant.get_route()
							#print ( "With the Route %s "  %self.best_route_seen)
							#print ( "With the Path %s "  %self.best_path_seen)
						dists.append(self.shortest_distance)
	
            	    ############################ANTS###############################
            	    
					#print ( "With the Route ", robot,  self.best_route_seen)                    
					#decay current pheromone values and add all pheromone values we saw during traversal (from ant_updated_pheromone_map)
					self.ph_map_update(robot)
            	    
            	    #Soft Assigning Tasks to specific Robots based on Pheromone Values 
					self.soft_assign(assign, robot_assign, robot)
		    	          
					#flag when first pass is done
					if self.first_pass:
						self.first_pass = False
					
					#reset all ANTS for next robot
					self.ants_init(self.start, robot)
					
					#reset ant_updated_pheromone_map to record pheromones for ants on next pass
					#print ("Main ", len(self.multi_robot_nodes))
					#self.ant_updated_pheromone_map.append(self.matrix_init(len(self.multi_robot_nodes)))
				#ROBOT#############################################################
				#print ("robot_assign ", robot_assign)                             
				s = 0
				for k in robot_assign:
					s = s + len(k)
            #WHILE#############################################################
            	    
			#translate shortest path back into callers node id's
			print ("robot_assign ", robot_assign)

			D, pp = self.rdp(robot_assign)
			print ("This Iteration Total Distrance is : ", D)
			#print ("The Corrresponding Path Plan is : ", pp)
			ret = []
			for id in self.best_route_seen:
				ret.append(self.id_to_key[id])
			route.append(ret)
        #ITERATION#############################################################
            
		return route, dists, self.best_path_seen

# =============================================================================
# Inputs Section:::
# =============================================================================

#Distance function to get distance between Tasks...
def distance(start, end):
	#print ("Distance Function")
	path_plan = []

	if start[1] is None:
		#first_pnd = 0
		traverse_dist, traverse_pp = path_plan_dist(start[0], end[0])
		path_plan.append(traverse_pp)
    
	else:
		''' First Task Distance (from pick to drop) + Second Task Distance + Inter-Task Distance'''
#		first_pnd, first_pp = path_plan_dist(start[0], start[1])
#		path_plan.append(first_pp)
		traverse_dist, traverse_pp = path_plan_dist(start[1], end[0])
		path_plan.append(traverse_pp)

	second_pnd, second_pp = path_plan_dist(end[0] , end[1])
   	path_plan.append(second_pp)

	#print (second_pnd + traverse_dist)
	return second_pnd + traverse_dist, path_plan
  
def euc_dist(start, end):
	x_distance = abs(start[0] - end[0])
	y_distance = abs(start[1] - end[1])
	return math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))

def man_dist(start, end):
    return abs(start[0] - end[0]) + abs(start[1] - end[1])

def path_plan_dist(start, end):
    #pp, _ = path( start, end, grid, [])
    #print("Grid ::: ", grid)
    pp = nx.astar_path(G, start, end, cost)
    #print (pp)
    path_length = len(pp)
    #print (path_length)
    #path.append(p)
    return float(path_length), pp

def cost(a, b):
    if map[a] >= 0 and map[b] >= 0:  # no obstacle
        #print np.linalg.norm(np.array(a)-np.array(b))
        return np.linalg.norm(np.array(a)-np.array(b))
    else:
        return np.Inf

jobs = [((7, 4), (0, 4), 4),
        ((2, 2), (3, 7), 3),
        ((4, 5), (7, 5), 0),
        ((4, 4), (6, 6), 1)]
test_nodes = { i : jobs[i] for i in range(0, len(jobs) ) }
#print ("These are the Tasks " , test_nodes)

#ROBOT STARTING POSTION #TODO
agent_pos = [(1, 5), (1, 6), (1, 1)]
robot_pos = []
for x in agent_pos:
    lst = []
    lst.append(x)
    lst.append(None)
    robot_pos.append(lst)
print robot_pos

#robot_pos=[(1,1), None]

#...we can make a colony of ants...
colony = ant_colony(test_nodes, distance, robot_pos)

#...that will find the optimal solution with ACO
aco_time = time.time()
answer, dists, path_plan = colony.main()
print "Best Route: " , answer
#print "Path Plan:  " , path_plan
print ("--- Time taken is %s seconds ---" % (time.time() - aco_time))
#print ("dists, " , dists)
plt.plot(dists)
plt.show()
