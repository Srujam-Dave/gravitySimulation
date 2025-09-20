from tkinter import *
import math
import numpy as np
from random import randint

#GLOBAL VARS
SCREEN_WIDTH = 900
EPSILON = 0.5
GRAVITY = 0.5
DELAY = 1
THETA = 0.5
draw_tree = True
rects = []
NUM_PLANETS = 250
FROZEN = False
MERGE = True
ELASTICITY = 1

#Contains methods for drawing each planet, updating its position, velocity,
#and acceleration, and dealing with collisions between planets
class Planet:
    #Initializes each planet (and draws it with the self.ball attribute)
    def __init__(self, position, velocity, mass):
        self.position = position
        self.velocity = velocity
        self.acceleration = np.array([0, 0])

        self.mass = mass
        self.radius = math.sqrt(self.mass) / 2
        self.ball = canvas.create_oval(self.position[0] - self.radius,
            self.position[1] - self.radius, self.position[0] + self.radius, 
            self.position[1] + self.radius, fill="white")

    #Updates position and velocity each frame
    def update_planet(self):
        self.position = np.add(self.velocity, self.position)
        self.velocity = np.add(self.acceleration, self.velocity)

    #Uses Quadtree to calculate force on each planet, adjusts acceleration
    def ApplyForces(self, root, planet_list, new_planets):
        total_force = root.calcForces(self, planet_list, new_planets)

        self.acceleration = total_force / self.mass
         
#Tree nodes used for implementing barnes-hut algorithm
class QuadTreeNode:
    def __init__(self, center, width):
        self.center = center #spatial center of node
        self.width = width #width of node from side to side
        self.mass = 0 #total mass of all planets in node
        self.planets = [] #list of planets in node
        self.com = np.array([0, 0]) #center of mass of node
        self.internal = False

        #children
        self.ne = None
        self.nw = None
        self.se = None
        self.sw = None

        #draw node outline for debugging purposes
        if draw_tree:
            rects.append(canvas.create_rectangle(
                int(center[0] - width / 2),  # left
                int(center[1] - width / 2),  # top
                int(center[0] + width / 2),  # right
                int(center[1] + width / 2),  # bottom
                outline="red"
            ))
    
    # add planet to tree and recursively to children
    def add_body(self,  planet, planet_list):
        #add offset so planets cannot have identical positions
        for p in self.planets: 
            if (np.array_equal(p.position, planet.position)):
                planet.position = planet.position + np.array([EPSILON, EPSILON])

        #update node attributes
        self.mass += planet.mass
        self.com = (planet.mass / self.mass) * planet.position + (1 - (planet.mass / self.mass)) * self.com

        #Distributes planets to leaf nodes
        if (self.internal):
            self.distribute_planet(planet, planet_list)
        else:
            #If planet is a leaf node, add planet and create new children if needed
            if (len(self.planets) == 0):
                self.planets.append(planet)
            else:
                for p in self.planets:
                    self.distribute_planet(p, planet_list)
                self.planets.clear()

                self.distribute_planet(planet, planet_list)

    #Creates leaf nodes and distributes planets among them as necessary
    def distribute_planet(self, planet, planet_list):
        self.internal = True
        offset = planet.position - self.center

        #if planet is out of bounds, remove it
        if (abs(offset[0]) > self.width/2 or abs(offset[1]) > self.width/2):
            if planet in planet_list: planet_list.remove(planet)
            canvas.delete(planet.ball )
            del planet
            return
        
        #find appropriate child to place planet in; create child if it doesn't exist
        if (offset[0] >= 0 and offset[1] >= 0):
            if (self.nw == None):
                self.nw = QuadTreeNode(self.center + np.array([self.width/4, self.width/4]), self.width/2)
            self.nw.add_body(planet, planet_list)
        elif (offset[0] >= 0 and offset[1] < 0):
            if (self.ne == None):
                self.ne = QuadTreeNode(self.center + np.array([self.width/4, -self.width/4]), self.width/2)
            self.ne.add_body(planet, planet_list)
        elif (offset[0] < 0 and offset[1] >= 0):
            if (self.sw == None):
                self.sw = QuadTreeNode(self.center + np.array([-self.width/4, self.width/4]), self.width/2)
            self.sw.add_body(planet, planet_list)
        elif (offset[0] < 0 and offset[1] < 0):
            if (self.se == None): 
                self.se = QuadTreeNode(self.center + np.array([-self.width/4, -self.width/4]), self.width/2)
            self.se.add_body(planet, planet_list)
    
    #Implements barnes-hut algorithm to approximate forces
    def calcForces(self, planet, planet_list, new_planets):
        #Calculate distance between node center of mass and planet
        dist_vect = self.com - planet.position
        distance = np.linalg.norm(dist_vect)

        #If current node contains just planet, there is no force, return 0
        if (distance == 0):
            return np.array([0.0, 0.0])
        
        
        if not self.internal and self.checkCollisions(planet, planet_list, new_planets):
                return np.array([0, 0])
        
        #If node is sufficiently far away/small, use approximation with node center of mass
        if (self.width / distance < THETA or not self.internal):
            #If the internal node is close enough that we're not approximating, check for collisions
            return dist_vect * (GRAVITY * self.mass * planet.mass / (distance ** 3))

        #If node is too close/too large, add up forces from all child nodes
        force = np.array([0.0, 0.0])
        if (self.nw != None): force += self.nw.calcForces(planet, planet_list, new_planets)
        if (self.ne != None): force += self.ne.calcForces(planet, planet_list, new_planets)
        if (self.se != None): force += self.se.calcForces(planet, planet_list, new_planets)
        if (self.sw != None): force += self.sw.calcForces(planet, planet_list, new_planets)

        return force

    #checks for collisions between planets
    def checkCollisions(nearNode, planet, planet_list, new_planets):
        for p in nearNode.planets:
            if (planet != p):
                #calculate distances and check for collision
                dist_vector = planet.position - p.position
                distance = math.sqrt(dist_vector[0] ** 2
                    + dist_vector[1] ** 2)
                
                if distance < (p.radius + planet.radius):

                    if MERGE:
                        #Position calculated by linearly interpolating between the
                        #positions of the previous two planets based on their 
                        #masses
                        new_pos = p.position + (dist_vector
                            * planet.mass / (planet.mass + p.mass))
                        new_mass = p.mass + planet.mass
                        #velocity calculated through conservation of momentum
                        new_vel = (p.velocity * p.mass
                            + planet.velocity * planet.mass) / new_mass
                        new_planet = Planet(new_pos, new_vel, new_mass)
                        
                        #Delete old two planets, their images, and add new one
                        canvas.delete(planet.ball)
                        canvas.delete(p.ball)

                        if p in planet_list: planet_list.remove(p)
                        if planet in planet_list: planet_list.remove(planet)

                        new_planets.append(new_planet)
                    else: 
                        p.velocity = -1 * p.velocity * ELASTICITY
                        planet.velocity = -1 * planet.velocity * ELASTICITY
                    return True
        return False

#function called every frame - reconstruct QuadTree, move planets, handle collisions
def move_planets():
    #delete old QuadTree drawing (for debugging)
    for rect in rects:
        canvas.delete(rect)

    #reconstruct QuadTree
    tree_root = QuadTreeNode(np.array([SCREEN_WIDTH/2, SCREEN_WIDTH/2]), SCREEN_WIDTH)
    for planet in planet_arr:
        tree_root.add_body(planet, planet_arr)

    if not FROZEN:
        #new planets created after collisions go here, added into tree right before next frame
        new_planets = []

        #apply forces and move planet
        for planet in planet_arr:
            planet.ApplyForces(tree_root, planet_arr, new_planets)
            planet.update_planet()
            canvas.move(planet.ball, planet.velocity[0], planet.velocity[1])
        
        #add new planets
        planet_arr.extend(new_planets)
    
    if (len(planet_arr) <2):
        reset_sim(None)

    #wait, go to next frame
    canvas.after(DELAY, move_planets)

#turns draw_tree on and off when user presses space bar
def toggleFrame(self):
    global draw_tree
    draw_tree = not draw_tree

#pause/unpause simulation
def togglePause(self):
    global FROZEN
    FROZEN = not FROZEN

#clears planet_arr and populates it with new planets
def reset_sim(self):
    canvas.delete("all")

    global planet_arr
    planet_arr.clear()


    planet_arr = []
    for i in range(0, NUM_PLANETS):
        newplanet = Planet(np.array([randint(0, SCREEN_WIDTH), randint(0, SCREEN_WIDTH)]),
            np.array([0, 0]), 10)
        planet_arr.append(newplanet)
    
    rects.clear()

#create root and canvas
root = Tk()
canvas = Canvas(root, bg="black", height=SCREEN_WIDTH, width=SCREEN_WIDTH)
canvas.pack()

#setup initial simulation
planet_arr = []
reset_sim(None)

#bind user controls
root.bind("<ButtonRelease>", toggleFrame)
root.bind("<space>", reset_sim)
root.bind("p", togglePause)
#main simulation loop
move_planets()
root.mainloop()
