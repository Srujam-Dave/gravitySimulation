from tkinter import *
import math
import numpy as np
from random import randint


GRAVITY = 0.25
DELAY = 10

#
class Planet:
    def __init__(self, position, velocity, mass):
        self.position = position
        self.velocity = velocity
        self.acceleration = np.array([0, 0])

        self.mass = mass
        self.radius = math.sqrt(self.mass) / 2
        self.ball = planet_canvas.create_oval(self.position[0] - self.radius,
            self.position[1] - self.radius, self.position[0] + self.radius, 
            self.position[1] + self.radius, fill="black")

    def UpdatePlanet(self):
        self.position = np.add(self.velocity, self.position)
        self.velocity = np.add(self.acceleration, self.velocity)

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

    def CheckCollisions(self, planets):
        for planet in planets:
            if planet != self:
                dist_vector = planet.position - self.position
                distance = math.sqrt(dist_vector[0] ** 2
                    + dist_vector[1] ** 2)
    
                if distance < (self.radius + planet.radius):
                    new_pos = self.position + (dist_vector
                        * planet.mass / (planet.mass + self.mass))
                    new_mass = self.mass + planet.mass
                    new_vel = (self.velocity * self.mass
                        + planet.velocity * planet.mass) / new_mass
                    new_planet = Planet(new_pos, new_vel, new_mass)
                    
                    planet_canvas.delete(planet.ball)
                    planet_canvas.delete(self.ball)
                    planets.remove(planet)
                    for i in range(len(planets)):
                        if planets[i] is self:
                            planets.remove(planets[i])
                            break

                    planets.append(new_planet)



def move_planets():
    for planet in planets:
        planet.ApplyForces(planets)
        planet.UpdatePlanet()
        planet.CheckCollisions(planets)
        planet_canvas.move(planet.ball, planet.velocity[0], planet.velocity[1])

    planet_canvas.after(DELAY, move_planets)


root = Tk()
planet_canvas = Canvas(root, bg="white", height=1000, width=1000)

planets = []

for i in range(0, 40):
    planets.append(Planet(np.array([randint(0, 1000), randint(0, 1000)]),
        np.array([randint(-1, 1), randint(-1, 1)]), randint(50, 200)))

planet_canvas.pack()
planet_canvas.after(DELAY, move_planets)
root.mainloop()
