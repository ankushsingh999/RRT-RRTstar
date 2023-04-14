# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 02:31:11 2023

@author: Ankush Singh
"""

# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math
import heapq


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
        self.distance = 0
        #for using heapqueue
    def __lt__(self, other):
        return self.distance < other.distance


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2
        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        #eucliadian distance
        d = math.sqrt(math.pow(node1.row-node2.row,2)+math.pow(node1.col-node2.col,2))
        return d

    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2
        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###
        check = 300
        #finding out the step increment
        incx = (node2.row - node1.row)/check
        incy = (node2.col - node1.col)/check
        #initializing the x and y points
        xpt = node1.row
        ypt = node1.col
        for i in range(check):
            #check if the point is an obstacle
            if self.map_array[int(xpt)][int(ypt)] == 0:
                return False           
            #incrementing x and y 
            xpt = xpt+incx
            ypt = ypt+incy
        return True



    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point
        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        #generating random points
        x = np.random.randint(0,self.size_row)
        y = np.random.randint(0,self.size_col)
        point = [int(x), int(y)]
        #goal bias should be between 5 to 10 percent
        #goal bias = 0.075
        #choosing between goal and point
        if np.random.uniform(0,1) > goal_bias:
            return point 
        else:
            return [self.goal.row,self.goal.col]
        
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point
        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        #declaring a queue calle ddistance
        distance = []
        for i in self.vertices:
            #finding the distance 
            d = math.sqrt(math.pow(i.row-point[0],2)+math.pow(i.col-point[1],2))
            #pushing the node in the queue based on the distance
            heapq.heappush(distance, (d,i))
        
        #popping the nearest node from the heap 
        least_d,near_node = heapq.heappop(distance)

        return near_node


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance
        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        #finding out the neghibors within a certain distance specified
        negh = [i for i in self.vertices if self.dis(i, new_node) < neighbor_size]        
        return negh

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node
        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        
        # Create a heap from the neighbors with costs as keys
        neighbor_heap = [(n.cost + self.dis(n, new_node), n) for n in neighbors]
        heapq.heapify(neighbor_heap)

        # Finding the neighbor with the minimum cost
        
        while True: 
            min_cost, min_cost_neighbor = heapq.heappop(neighbor_heap)
            #check for the collision of the node
            if self.check_collision(min_cost_neighbor, new_node) == True:
                new_node.cost = min_cost
                new_node.parent = min_cost_neighbor 
            break

        # Rewire the remaining neighbors
        while neighbor_heap:
            cost, neighbor = heapq.heappop(neighbor_heap)
            potential_cost = new_node.cost + self.dis(new_node, neighbor) 
            if neighbor.cost > potential_cost and self.check_collision(neighbor, new_node) == True:
                neighbor.parent = new_node
                neighbor.cost = potential_cost

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()
        
    def get_direction(self,new,negh):
        #get the direction of the node to extend the tree
        x = new[0]-negh.row
        y = new[1]-negh.col
        direction=np.arctan2(y,x)
        return direction
    
    def check_valid(self,point):
        #check the validity of the new nodw created if it is in the graph
        if point[0] >= self.size_row:
            point[0] = self.size_row-1
        if point[1] >= self.size_col:
            point[1] = self.size_col-1
        return point

    def generate_directed_point(self,negh,direct,ext_step):
        point = [0,0]
        point[0]=negh.row+ext_step*np.cos(direct)
        point[1]=negh.col+ext_step*np.sin(direct)
        return [point[0],point[1]]

    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.
        self.start.parent = self.start
        self.start.cost = 0
        ext_step = 10
        #goal bias taken between 5 to 10 percent
        goal_bias = 0.075
        for i in range(n_pts):
            #generating random points
            point=self.get_new_point(goal_bias)
            if self.map_array[point[0]][point[1]]==0:
                continue
            negh=self.get_nearest_node(point)
            new = [0,0]
            
            #get nodes in the direction of the new node
            direct = self.get_direction(point,negh)
            [new[0],new[1]] = self.generate_directed_point(negh,direct,ext_step)
      
            npoint = self.check_valid(new)

            if self.map_array[int(npoint[0])][int(npoint[1])]==0:
                continue
            NewNode=Node(npoint[0],npoint[1])
            
            if self.check_collision(negh,NewNode)==True:
                disnn = self.dis(NewNode,negh)
                NewNode.parent=negh
                NewNode.cost=negh.cost+disnn
                
                self.vertices.append(NewNode)
                
            else:
                continue
            dgoal = self.dis(NewNode,self.goal)
            if dgoal < 8:
                self.found=True
                self.goal.parent=NewNode
                self.goal.cost=NewNode.cost+self.dis(NewNode,self.goal)
                self.vertices.append(self.goal)
                break
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")
        
        # Draw result
        self.draw_map()


    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        self.start.parent=self.start
        self.start.cost=0
        ext_step = 15
        goal_bias = 0.075
        for i in range(n_pts):
            point=self.get_new_point(goal_bias)
            if self.map_array[point[0]][point[1]]==0:
                continue
            negh=self.get_nearest_node(point)
            new = [0,0]
            direct = self.get_direction(point,negh)
            [new[0],new[1]] = self.generate_directed_point(negh,direct,ext_step)
                
            npoint = self.check_valid(new)
            
            if self.map_array[int(npoint[0])][int(npoint[1])]==0:
                continue
            NewNode=Node(npoint[0],npoint[1])
            dissn = self.dis(NewNode,negh)
            if self.check_collision(negh,NewNode)==True:
                
                NewNode.parent=negh
                NewNode.cost=negh.cost+dissn
                
                neighbour=self.get_neighbors(NewNode, neighbor_size)

                if len(neighbour)==0:
                    continue
                self.rewire(NewNode,neighbour)
                self.vertices.append(NewNode)
            else:
                continue
            dgoal = self.dis(NewNode,self.goal)
            if dgoal < 8:
                
                self.found=True
                
                self.goal.parent=NewNode
                disg = self.dis(NewNode,self.goal)
                self.goal.cost=NewNode.cost+disg
                neighbour_new=self.get_neighbors(self.goal, neighbor_size)
                if len(neighbour_new)==0:
                    continue
                self.rewire(self.goal,neighbour_new)
                self.vertices.append(self.goal)
                
                

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()