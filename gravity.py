from tkinter import *
import math
import numpy as np
from random import randint

#GLOBAL VARIABLES
GRAVITY = 0.25
DELAY = 10

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
        self.ball = planet_canvas.create_oval(self.position[0] - self.radius,
            self.position[1] - self.radius, self.position[0] + self.radius, 
            self.position[1] + self.radius, fill="black")

    #Updates position and velocity each frame
    def UpdatePlanet(self):
        self.position = np.add(self.velocity, self.position)
        self.velocity = np.add(self.acceleration, self.velocity)

    #Calculate the force of gravity from other planets on this planet
    def ApplyForces(self, planets):
        total_force = np.array([0, 0])
        for planet in planets:
            if planet != self:
                dist_vector = planet.position - self.position
                distance = math.sqrt(dist_vector[0] ** 2 
                    + dist_vector[1] ** 2)

                force = (dist_vector
                    * (GRAVITY * self.mass * planet.mass / (distance ** 3)))
                total_force = total_force + force

        self.acceleration = total_force / self.mass

    #If there's a collision between two planets, deletes both planets and
    #creates a new one with the mass of both combined
    def CheckCollisions(self, planets):
        #Iterates through every other planet and checks for collisions
        for planet in planets:
            if planet != self:
                dist_vector = planet.position - self.position
                distance = math.sqrt(dist_vector[0] ** 2
                    + dist_vector[1] ** 2)
                
                if distance < (self.radius + planet.radius):
                    #Position calculated by linearly interpolating between the
                    #positions of the previous two planets based on their 
                    #masses
                    new_pos = self.position + (dist_vector
                        * planet.mass / (planet.mass + self.mass))
                    new_mass = self.mass + planet.mass
                    #velocity calculated through conservation of momentum
                    new_vel = (self.velocity * self.mass
                        + planet.velocity * planet.mass) / new_mass
                    new_planet = Planet(new_pos, new_vel, new_mass)
                    
                    #Delete old two planets, their images, and add new one
                    planet_canvas.delete(planet.ball)
                    planet_canvas.delete(self.ball)
                    planets.remove(planet)
                    for i in range(len(planets)):
                        if planets[i] is self:
                            planets.remove(planets[i])
                            break
                    planets.append(new_planet)


#Stores all the planets in the simulation and contains methods for resetting
#the simulation, adjusting it, and moving the simulation forwards
class PlanetCluster:
    def __init__(self):
        self.cluster = [];
        self.total_mass = 0;

    #Populate array (and screen) with a bunch of planet objects
    def create_planets(self):
        self.cluster.clear()
        planet_canvas.delete("all")
        self.total_mass = 0

        for i in range(0, randint(30, 50)):
            mass = randint(50, 200)
            self.cluster.append(Planet(np.array([randint(0, 1000), randint(0, 1000)]),
                np.array([randint(-1, 1), randint(-1, 1)]), mass))
        self.total_mass += mass


    #Uses recursion to update each planet's attributes, move it, and check for 
    #collisions, then go to the next frame
    def move_planets(self):
        for planet in self.cluster:
            planet.ApplyForces(self.cluster)
            planet.UpdatePlanet()
            planet.CheckCollisions(self.cluster)
            planet_canvas.move(planet.ball, planet.velocity[0], planet.velocity[1])

        planet_canvas.after(DELAY, self.move_planets)


#Create window and canvas
root = Tk()
planet_canvas = Canvas(root, bg="white", height=1000, width=1000)
planet_canvas.pack()

#populates window with lots of small planets
planet_cluster = PlanetCluster()
planet_cluster.create_planets()

#Resets simulation if mouse pressed
def reset_simulation(self):
    planet_cluster.create_planets()

root.bind("<ButtonRelease>", reset_simulation)

planet_canvas.after(DELAY, planet_cluster.move_planets)
root.mainloop()
