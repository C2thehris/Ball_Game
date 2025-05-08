import pygame
import sys
import numpy as np
import random
import itertools
import math
from Alex_Vector_Functions import *

pygame.init() 
pygame.display.set_caption('ball game')
w, h = pygame.display.Info().current_w*4/5, pygame.display.Info().current_h*4/5
screen_dimensions = [w, h]
screen = pygame.display.set_mode((screen_dimensions), pygame.RESIZABLE, vsync=1)
fpsClock = pygame.time.Clock()

#global constants
FPS = 120 #framerate (1/s)
g = np.array([0, 32], dtype = float) #gravity (m/s^2)
pixels_per_meter = 100 #(px/m)
w, h = w/pixels_per_meter, h/pixels_per_meter
mouse_radius = 0.2 #m
air_density = 1.225 #kg/m^3

#variable initial conditions
counter = 0
t = 0 #s
moving_vertex = False

def draw_dashed_line(surface, color, start_pos, end_pos, dash_length=10):
    """Draws a dashed line from start_pos to end_pos."""
    x1, y1 = start_pos
    x2, y2 = end_pos
    dx, dy = x2 - x1, y2 - y1
    distance = (dx ** 2 + dy ** 2) ** 0.5
    dash_count = int(distance / dash_length)
    
    for i in range(dash_count):
        start_x = x1 + (dx / dash_count) * i
        start_y = y1 + (dy / dash_count) * i
        end_x = x1 + (dx / dash_count) * (i + 0.5)
        end_y = y1 + (dy / dash_count) * (i + 0.5)
        pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), 2)

def possible_collisions(combinations): #Returns a list of possible collisions based on if players' projections on the x axis and y axis overlap.
    possibilities_x = []
    for combination in combinations:
        left_0 = combination[0].position[0] - combination[0].radius
        right_0 = combination[0].position[0] + combination[0].radius
        left_1 = combination[1].position[0] - combination[1].radius
        right_1 = combination[1].position[0] + combination[1].radius
        if (right_0 > left_1 and right_0 < right_1) or (right_1 > left_0 and right_1 < right_0):
            possibilities_x.append(combination) 
    possibilities_y = []
    for combination in possibilities_x:
        bottom_0 = combination[0].position[1] - combination[0].radius
        top_0 = combination[0].position[1] + combination[0].radius
        bottom_1 = combination[1].position[1] - combination[1].radius
        top_1 = combination[1].position[1] + combination[1].radius
        if (top_0 > bottom_1 and top_0 < top_1) or (top_1 > bottom_0 and top_1 < top_0):
            possibilities_y.append(combination) 
    return possibilities_y

class Player: #draws and moves player around
    def __init__(self, position, radius, color, density = 1000, Cd = 0.47):
        self.position, self.radius, self.color = np.array(position, dtype = float), radius, color #m, m, string
        self.previous_position = self.position #m
        self.velocity, self.acceleration = np.array([0, 0], dtype = float), np.array([0, 0], dtype = float) #m/s, m/s^2
        self.bounciness, self.slipperiness = 0.70, 0.999 #coefficient of restitution
        self.density = density #kg/m^2
        self.mass = self.density * math.pi * self.radius**2 #kg
        self.theta, self.omega, self.moment_of_inertia = 0, 0, 0.5 * self.mass * self.radius**2 #rad, rad/s, kgm^2
        self.touching_obstacle = 0
        self.touching_player = 0
        self.area = math.pi * self.radius ** 2 #cross sectional area (m^2)
        self.Cd = Cd # drag coefficient no units
        
    def draw(self): #draws player
        n = 2
        for i in range(1, n+1):
            bisecting_vector = self.radius * np.array([math.cos(self.theta + i * math.pi/n), math.sin(self.theta + i * math.pi/n)]) #m
            bisecting_line = ((self.position + bisecting_vector)*pixels_per_meter, (self.position - bisecting_vector)*pixels_per_meter) #m
            pygame.draw.line(screen, "green", bisecting_line[0], bisecting_line[1], 2)
        pygame.draw.circle(screen, self.color, self.position*pixels_per_meter, self.radius*pixels_per_meter, 2)
    
    def kinematics(self, iterations): #moves player around
        #dyanmics
        rocket_force = 50 * self.mass  * Player.inputs()
        weight = self.mass * g
        drag_force = - 0.5 * self.Cd * self.area * air_density * unit_vector(self.velocity) * magnitude(self.velocity)**2 #drag force (N)
        self.net_force = rocket_force + weight + drag_force #sum of force (N)

        #kinematics
        dt = 1/(FPS * iterations) #time step (s)
        self.acceleration = self.net_force / self.mass #acceleration = rocket force + gravity + drag      
        self.velocity += self.acceleration * dt #change in velocity (m/s)
        self.position += self.velocity * dt #change in position (m)
        self.previous_position = self.position - self.velocity * dt
        self.omega -= 0.01 * self.omega / iterations #change in angular speed (rad/s)
        self.theta += self.omega / iterations #change in angle (rad)

    def collide(self, other_player): #bounces players off of of eachother using conservation out of momentum, bounciness, and slipperiness 
        total_radius = self.radius + other_player.radius #m
        displacement = other_player.position - self.position #m
        distance = magnitude(displacement) #m

        if distance < total_radius:
            self.touching_player += 1
            adder_1, adder_2 = 0, 0
            if self.touching_obstacle > 0:
                adder_1 -= 0.5
                adder_2 += 0.5
            if other_player.touching_obstacle > 0:
                adder_1 += 0.5
                adder_2 -= 0.5

            overlap = (1) * (total_radius - distance)
            direction = unit_vector(displacement)
            other_player.position += (adder_2 + 0.5) * overlap * direction #reposition players away from eachother after colliding
            self.position -= (adder_1 + 0.5) * overlap * direction 
            
            u1, u2 = other_player.velocity, self.velocity
            u_approach = u2 - u1 #velocity of approach
            u_normal, u_tangential = components(u_approach, displacement)
            v_separation = self.slipperiness * u_tangential - self.bounciness * u_normal #velocity of separation
            m1, m2 = other_player.mass, self.mass
            other_player.velocity = (m1*u1 + m2*u2 - m2*v_separation)/(m1 + m2) #conservation of momentum
            self.velocity = (m1*u1 + m2*u2 + m1*v_separation)/(m1 + m2)

    def rotation_collide(self, other_player):
        total_radius = self.radius + other_player.radius
        displacement = other_player.position - self.position
        distance = magnitude(displacement)

        if distance < total_radius:
            m1, m2 = other_player.mass, self.mass
            omega_1, omega_2 = other_player.omega, self.omega
            r_1, r_2 = other_player.radius, self.radius
            I1, I2 = 0.5 * m1 * r_1 ** 2, 0.5 * m2 * r_2 ** 2 #moment of inertia
            angular_momentum = I1 * omega_1 + I2 * omega_2

            omega_approach = omega_2-omega_1
            omega_separation = -1 * omega_approach
            if magnitude(other_player.velocity) > .0:
                other_player.omega = (angular_momentum - omega_separation * I2) / (I1 + I2)
            else: other_player.omega = 0
            if magnitude(self.velocity) > .0:
                self.omega = (angular_momentum + omega_separation * I1) / (I1 + I2)
            else: self.omega = 0

            # prevents balls from spinning when multiple are stacked on top of each other and are at rest
            '''if (self.touching_obstacle > 0 and self.touching_player > 0) or self.touching_player > 1 or self.touching_obstacle > 1:
                self.omega = 0
            if (other_player.touching_obstacle > 0 and other_player.touching_player > 0) or other_player.touching_player > 1 or other_player.touching_obstacle > 1:
                other_player.omega = 0'''

    def inputs(): #returns a vector for the direction the arrow keys are being pressed
        direction = np.array([0, 0], dtype = float) 
        ratio = 0.5
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: direction += np.array([0, -1], dtype = float)
        if keys[pygame.K_DOWN]: direction += np.array([0, 1], dtype = float)
        if keys[pygame.K_LEFT]: direction += np.array([-ratio, 0], dtype = float)
        if keys[pygame.K_RIGHT]: direction += np.array([ratio, 0], dtype = float)
        return direction #unit_vector(direction)

class Circle: 
    def __init__(self, position, radius, color, collision): #position, vector, color, has collision?
        self.position = np.array(position, dtype = float)
        self.radius = radius
        self.color = color
        self.collision = collision
    
    def kinematics(self):
        pass

    def draw(self): #draws circle
        if self.radius > 0: pygame.draw.circle(screen, self.color, self.position*pixels_per_meter, self.radius*pixels_per_meter, 2)

    def overlap(self, my_player): #determines if player overlaps circle
        distance = np.linalg.norm(my_player.position - self.position)
        if distance < my_player.radius + self.radius: return True
        else: return False
    
    def collide(self, my_player): #bounces player off circle
        if self.collision and self.overlap(my_player): 
            my_player.touching_obstacle += 1

            total_radius = my_player.radius + self.radius
            displacement = my_player.position - self.position
            v_normal, v_tangent = components(my_player.velocity, displacement)
            my_player.position = self.position + total_radius * unit_vector(displacement)
            my_player.velocity = my_player.slipperiness * v_tangent - my_player.bounciness * v_normal
            
            cross_product = v_tangent[0] * displacement[1] - v_tangent[1] * displacement[0]
            if cross_product > 0: sign = -1
            else: sign = 1
            scaler = 1
            if my_player.touching_obstacle == 1:
                my_player.omega = scaler * sign * np.linalg.norm(v_tangent) / (my_player.radius * pixels_per_meter)
            else: my_player.omega = 0
            
class Line: 
    def __init__(self, position, vector, color, collision): #position, vector, color, has collision?
        self.position = np.array(position, dtype = float)
        self.vector = np.array(vector, dtype = float)
        self.length = np.linalg.norm(self.vector)
        self.color = color
        self.collision = collision
        self.r = 0
        self.circle_tracker = Circle(self.position, self.r, self.color, self.collision)
        self.circle_beg = Circle(self.position, self.r, self.color, self.collision)
        self.circle_end = Circle(self.position + self.vector, self.r, self.color, self.collision)

    def kinematics(self):
        pass
    
    def draw(self): #draws line
        pygame.draw.line(screen, self.color, self.position*pixels_per_meter, (self.position + self.vector)*pixels_per_meter, 2)
        if self.r != 0:
            self.circle_tracker.draw()
            self.circle_beg.draw()
            self.circle_end.draw()

    def overlap(self, my_player): #determines if player overlaps with line
        displacement = my_player.position - self.position
        tangential_displacement, normal_displacement = components(displacement, self.vector)
        tangential_distance, normal_distance = magnitude(tangential_displacement), magnitude(normal_displacement)
        over_line = np.dot(tangential_displacement, self.vector) > 0 and tangential_distance < self.length
        if over_line and normal_distance < my_player.radius: return True
        else: return False

    def prevent_clipping(self, my_player):
        previous_displacement, displacement = my_player.previous_position - self.position, my_player.position - self.position
        tangential_displacement, normal_displacement = components(displacement, self.vector)
        tangential_distance, normal_distance = magnitude(tangential_displacement), magnitude(normal_displacement)
        if normal_distance < 1:
            over_line = np.dot(tangential_displacement, self.vector) > 0 and tangential_distance < self.length
            if np.cross(previous_displacement, self.vector) < 0: sign_1 = -1
            else: sign_1 = 1
            if np.cross(displacement, self.vector) < 0: sign_2= -1
            else: sign_2 = 1
            if sign_1 != sign_2 and over_line:
                my_player.position = self.position + tangential_displacement - (my_player.radius / 2) * unit_vector(normal_displacement)

    def collide(self, my_player): #bounces player off of line
        self.circle_beg.collide(my_player) #ball bounces off ends of line
        self.circle_end.collide(my_player)
        #self.prevent_clipping(my_player) #prevents ball from clipping through line

        if self.collision and self.overlap(my_player): #ball bounces off line if they overlap
            displacement = my_player.position - self.position
            tangential_displacement = projection(displacement, self.vector)
            self.circle_tracker.position = self.position + tangential_displacement
            self.circle_tracker.collide(my_player)
           
class Polyline: #vectors defines polyline with coordinates relative to the end of the previous line segment, verticies define polylines with coordinates relative to the origin
    def __init__(self, vectors, color, collision = True): 
        self.color = color
        self.collision = collision
        self.vertices = [np.array(vectors[0], dtype=float)]
        self.vectors = []
        for i in range(len(vectors)):
            self.vectors.append(np.array(vectors[i], dtype = float))
        self.lines = []
        self.vertices = [self.vectors[0]]
        for i in range(1, len(vectors)):
            self.lines.append(Line(self.vertices[-1], np.array(vectors[i], dtype = float), self.color, self.collision))
            self.vertices.append(self.vertices[i-1] + vectors[i])   

    def change_vertex(self, vertex, index):
        self.vertices[index] = np.array(vertex, dtype = float)
        #change line before vertex
        if index > 0: 
            self.vectors[index] = self.vertices[index] - self.vertices[index-1]
            self.lines[index-1] = Line(self.vertices[index-1], self.vectors[index], self.color, self.collision)
        else: 
            self.vectors[index] = self.vertices[index]

        #change line after vertex
        if index < len(self.vertices) - 1:
            self.vectors[index + 1] = self.vertices[index+1] - self.vertices[index]
            self.lines[index] = Line(self.vertices[index], self.vectors[index+1], self.color, self.collision)

    def mouse_near_vertex(self, mouse_radius):
        for i in range(len(self.vertices)):
            if magnitude(np.array(pygame.mouse.get_pos(), dtype = float)/pixels_per_meter - self.vertices[i]) < mouse_radius:
                return i 
        return -1
    
    def draw_divider(self, index):
        divider = 10*unit_vector(-unit_vector(self.vectors[index+1]) + unit_vector(self.vectors[index]))
        #pygame.draw.line(screen, "black", (self.vertices[index]-divider)*pixels_per_meter, (self.vertices[index] + divider)*pixels_per_meter, 2)
        draw_dashed_line(screen, "black", (self.vertices[index]-divider)*pixels_per_meter, (self.vertices[index] + divider*0)*pixels_per_meter, dash_length=300)
        draw_dashed_line(screen, "black", (self.vertices[index]+divider)*pixels_per_meter, (self.vertices[index] + divider*0)*pixels_per_meter, dash_length=300)

    def kinematics(self):
        pass

    def overlap(self, my_player): #determines if player overlaps polyline
        for line in self.lines:
            if line.overlap(my_player): return True
        return False
        
    def collide(self, my_player): #bounces player off of line
        for line in self.lines:
            line.collide(my_player)

    def draw(self): #draws polyline
        for line in self.lines:
            line.draw()

#COR = Center of Rotation, radius = circle radius, orbit_radius = distance from circle center to COR, theta = orbit angle, omega = orbit speed
class Spinning_Circle:
    def __init__(self, COR, radius, orbit_radius, theta, omega, color, collision):
        self.color = color
        self.collision = collision
        self.radius = radius
        self.COR = COR
        self.orbit_radius = orbit_radius
        self.theta = theta
        self.omega = omega
        self.position = 0
        self.COR_velocity = 0
    
    def kinematics(self):
        self.theta += self.omega * dt
        #self.COR += self.COR_velocity * dt
        self.position = np.array([self.COR[0] + self.orbit_radius*math.cos(self.theta), self.COR[1] + self.orbit_radius*math.sin(self.theta)], dtype = float)
    
    def draw(self): #draws circle
        self.kinematics()
        if self.radius > 0: pygame.draw.circle(screen, self.color, self.position*pixels_per_meter, self.radius*pixels_per_meter, 2)

    def overlap(self, my_player): #determines if player overlaps circle
        distance = np.linalg.norm(my_player.position - self.position)
        if distance < my_player.radius + self.radius: return True
        else: return False
    
    def collide(self, my_player):
        if self.collision and self.overlap(my_player): 
            #before collision
            my_player.touching_obstacle += 1
            displacement = my_player.position - self.position
            self.velocity = (self.omega * self.orbit_radius * np.array([-math.sin(self.theta), math.cos(self.theta)], dtype = float) + self.COR_velocity * iterations**2) / iterations
            self.normal_velocity, self.tangential_velocity = components(self.velocity, displacement)
            total_radius = my_player.radius + self.radius
            displacement = my_player.position - self.position
            v_normal, v_tangent = components(my_player.velocity, displacement)

            #after collision
            v_normal = self.normal_velocity + my_player.bounciness*(self.normal_velocity-v_normal)
            v_tangent = self.tangential_velocity - my_player.slipperiness*(self.tangential_velocity-v_tangent)
            my_player.position = self.position + total_radius * unit_vector(displacement)
            my_player.velocity = v_normal+v_tangent
            cross_product = v_tangent[0] * displacement[1] - v_tangent[1] * displacement[0]
            if cross_product > 0: sign = -1
            else: sign = 1
            scaler = 1
            if my_player.touching_obstacle == 1:
                my_player.omega = scaler * sign * np.linalg.norm(v_tangent) / (my_player.radius * pixels_per_meter)
            else: my_player.omega = 0

class Spinning_Line: #COR = center of rotation, #position = Cor to beginning of line, vector = beginning to end of line, omega = rotation speed
    def __init__(self, COR, position, vector, omega, color, collision): #beginning and end vectors are relative to orbit center
        self.r = 0
        self.beginning_vector = np.array(position, dtype = float)
        self.end_vector = np.array(position, dtype = float) + np.array(vector, dtype = float)
        self.line_vector = self.end_vector - self.beginning_vector
        self.length = magnitude(self.line_vector) #m
        self.color = color #string
        self.collision = collision #boolean 
        self.COR = COR #m
        self.omega = omega #rad/s
        self.theta_beg = get_angle(self.beginning_vector)
        self.theta_end = get_angle(self.end_vector)
        self.distance_beg, self.distance_end = magnitude(self.beginning_vector), magnitude(self.end_vector)
        self.circle_tracker = Spinning_Circle(self.COR, self.r, 0, 0, self.omega, self.color, self.collision)
        self.circle_beg = Spinning_Circle(self.COR, self.r, self.distance_beg, self.theta_beg, self.omega, self.color, self.collision)
        self.circle_end = Spinning_Circle(self.COR, self.r, self.distance_end, self.theta_end, self.omega, self.color, self.collision)
        self.COR_velocity = 0

    def kinematics(self):
        self.theta_beg += self.omega * dt
        self.theta_end += self.omega * dt
        #self.COR += self.COR_velocity * dt
        self.circle_tracker.COR_velocity = self.COR_velocity
        self.circle_tracker.COR = self.COR
        self.circle_beg.COR_velocity = self.COR_velocity
        self.circle_beg.COR = self.COR
        self.circle_end.COR_velocity = self.COR_velocity
        self.circle_end.COR = self.COR
        self.distance_beg, self.distance_end = magnitude(self.beginning_vector), magnitude(self.end_vector)
        self.beginning_vector = np.array([self.distance_beg*math.cos(self.theta_beg), self.distance_beg*math.sin(self.theta_beg)], dtype = float)
        self.end_vector = np.array([self.distance_end*math.cos(self.theta_end), self.distance_end*math.sin(self.theta_end)], dtype=float)
        self.line_vector = self.end_vector - self.beginning_vector

    def draw(self):
        self.kinematics()
        self.circle_beg.draw()
        self.circle_end.draw()
        pygame.draw.line(screen, self.color, (self.COR + self.beginning_vector)*pixels_per_meter, (self.COR + self.end_vector)*pixels_per_meter, 2)

    def overlap(self, my_player): #determines if player overlaps with line
        displacement = my_player.position - (self.beginning_vector + self.COR)
        tangential_displacement, normal_displacement = components(displacement, self.line_vector)
        tangential_distance, normal_distance = magnitude(tangential_displacement), magnitude(normal_displacement)
        over_line = np.dot(tangential_displacement, self.line_vector) > 0 and tangential_distance < self.length
        if over_line and normal_distance < my_player.radius + self.r: return True
        else: return False
    
    def prevent_clipping(self, my_player):
        previous_displacement, displacement = my_player.previous_position - self.beginning_vector, my_player.position - self.beginning_vector
        tangential_displacement, normal_displacement = components(displacement, self.line_vector)
        tangential_distance, normal_distance = magnitude(tangential_displacement), magnitude(normal_displacement)
        if normal_distance < 1:
            over_line = np.dot(tangential_displacement, self.line_vector) > 0 and tangential_distance < self.length
            if np.cross(previous_displacement, self.line_vector) < 0: sign_1 = -1
            else: sign_1 = 1
            if np.cross(displacement, self.line_vector) < 0: sign_2= -1
            else: sign_2 = 1
            if sign_1 != sign_2 and over_line:
                my_player.position = self.beginning_vector + tangential_displacement - (my_player.radius / 2) * unit_vector(normal_displacement)

    def collide(self, my_player):
        if self.collision and self.overlap(my_player):
            displacement = my_player.position - (self.beginning_vector + self.COR)
            tangential_displacement = projection(displacement, self.line_vector)
            contact_vector = tangential_displacement + self.beginning_vector
            self.circle_tracker.orbit_radius = magnitude(contact_vector)
            self.circle_tracker.theta = get_angle(contact_vector)
            self.circle_tracker.draw()
            self.circle_tracker.collide(my_player)

        self.prevent_clipping(my_player)
        self.circle_beg.orbit_radius, self.circle_beg.theta = self.distance_beg, self.theta_beg 
        self.circle_end.orbit_radius, self.circle_end.theta = self.distance_end, self.theta_end 
        self.circle_beg.collide(my_player)
        self.circle_end.collide(my_player)

class Spinning_Polyline:
    def __init__(self, vectors, omega, color, collision, COR_velocity = 0, amplitude = (2, 1), period = (2, 1), phase_shift = (0, 0), pause_time = (0, 0)):
        self.vectors = []
        for vector in vectors:
            self.vectors.append(np.array(vector, dtype = float))
        self.COR_start = np.array(vectors[0], dtype = float)
        self.COR = self.COR_start
        self.position = np.array(vectors[1], dtype = float)
        self.color = color
        self.collision = collision
        my_position = self.position
        self.lines = []
        for i in range(2, len(self.vectors)):
            self.lines.append(Spinning_Line(self.COR, my_position, self.vectors[i], omega, self.color, self.collision))
            my_position += vectors[i]    
        self.COR_velocity = COR_velocity
        self.amplitude = amplitude
        self.period = period
        self.pause_time = pause_time
        self.pause = False
        self.phase_shift = phase_shift
        self.adjusted_time = 0
        self.pause_time = pause_time
        self.time_paused = 0

    def kinematics(self):
        if self.pause:
            self.time_paused += dt
            if self.time_paused >= self.pause_time[0]:
                self.time_paused = 0
                self.pause = False
        else:
            self.adjusted_time += dt
            if self.adjusted_time > 0 and self.adjusted_time * 2 % self.period[0] <= 2 * dt:
                self.pause = True

        self.COR = np.array([self.COR_start[0] + self.amplitude[0]*math.sin(self.phase_shift[0] + 2*math.pi*self.adjusted_time/self.period[0]), self.COR_start[1] + self.amplitude[1]*math.sin(self.phase_shift[1] + 2*math.pi*t/self.period[1])], dtype = float)
        self.prev_COR = np.array([self.COR_start[0] + self.amplitude[0]*math.sin(self.phase_shift[0] + 2*math.pi*(self.adjusted_time-dt)/self.period[0]), self.COR_start[1] + self.amplitude[1]*math.sin(self.phase_shift[1] + 2*math.pi*(t-dt)/self.period[1])], dtype = float)
        self.COR_velocity = (self.COR - self.prev_COR) / (dt * iterations ** 2)
        for line in self.lines:
            line.COR_velocity = self.COR_velocity
            line.COR = self.COR

    def overlap(self, my_player): #determines if player overlaps polyline
        for line in self.lines:
            if line.overlap(my_player): return True
        return False
        
    def collide(self, my_player): #bounces player off of line
        for line in self.lines:
            line.collide(my_player)

    def draw(self): #draws polyline
        self.kinematics()
        for line in self.lines:
            line.draw()
        pygame.draw.circle(screen, "red", self.COR*pixels_per_meter, 5)

    def mouse_edit(self):
        pass

class Button:
    def __init__(self, position_1, position_2, text):
        self.position_1 = np.array(position_1, dtype=float)
        self.position_2 = np.array(position_2, dtype=float)
        self.text = text

    def draw(self):
        pygame.draw.rect(screen, "red", (pixels_per_meter * self.position_1, pixels_per_meter * self.position_2))
        font = pygame.font.Font(None, 36)
        text = "Hello, Pygame!"
        text_surface = font.render(text, True, "black")
        text_rect = text_surface.get_rect(center=(self.position_1+np.array((1, 0.5), dtype=float))*pixels_per_meter/1)
        screen.blit(text_surface, text_rect)

players = []
n = 4
for i in range(n):
    players.append(Player((2 + i * 4/n, 1), .5 + .0 * i, "black"))

player_combinations = []
for x, y in itertools.combinations(players, 2):
    player_combinations.append((x, y))

def speen(vectors, n):
    theta = -2 * math.pi / n
    all_vectors = vectors
    for i in range(n):
        for j in range(len(vectors)):
            vectors[j] = rotate_vector(vectors[j], theta)
        all_vectors += [vectors[j]]
    return all_vectors

buttons = [Button((0.5, 0.5), (2, 1), "hi"),
           Button((3.0, 0.5), (2, 1), "hi"),
           Button((5.5, 0.5), (2, 1), "hi")]

obstacles = [Polyline(((0, 4), ((w-13)/2, 0), (3, 3.25), (7, 0), (3, -3.25), ((w-13)/2, 0)), "black", True),
            #Polyline(((w , 0), (0, h ), (-w , 0), (0, -h), (w, 0)), "black", True),
            Polyline(((0, 0), (0, h ), (w , 0), (0, -h), (-w, 0)), "black", True),
            Spinning_Polyline(((w/2, h/2), (-.25, -.25), (0, -2), (0.5, .25), (0, 1.75), (2, 0), (-.25, .5), (-1.75, 0), (0, 2), (-0.5, -.25), (0, -1.75), (-2, 0), (.25, -.5), (1.75, 0)), -3*math.pi + .01, "red", True)]

def velocities(max_speed, theta, period, index = 0):
    global t
    speed = max_speed*math.cos(t*2*math.pi/(period))
    return speed * np.array([math.cos(theta), math.sin(theta)], dtype = float)

moving_vertex=False

while True: #game loop
    screen.fill("white")
    iterations = 2
    dt = 1 / (FPS * iterations)
    t += dt

    #for button in buttons:
        #button.draw()

    #calculating all collisions
    for i in range(iterations):
        for my_player in players:
            my_player.kinematics(iterations) #lets user move player + gravity + air resistance

        possibilities = possible_collisions(player_combinations) #player_combinations
        for possibility in possibilities: #player to player collisions
            possibility[0].touching_player, possibility[1].touching_player = 0, 0
            possibility[0].collide(possibility[1])

        for my_player in players:
            my_player.touching_obstacle = 0
            for obstacle in obstacles: #player to obstacle collisions
                obstacle.collide(my_player)
        
        for possibility in possibilities: #player to player collisions
            possibility[0].rotation_collide(possibility[1])   

    #drawing frame and checking if player quits
    for my_player in players:
        my_player.draw() #draws player
    for obstacle in obstacles:
        obstacle.draw() #draws obstacle

    ind_2 = obstacles[0].mouse_near_vertex(mouse_radius)
    if ind_2 > 0:
        pygame.draw.circle(screen, obstacles[0].color, obstacles[0].vertices[ind_2]*pixels_per_meter, .05*pixels_per_meter)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        #lets mouse drag polyline vertex
        pos = pygame.mouse.get_pos()
        if event.type == pygame.MOUSEBUTTONDOWN and moving_vertex == False:
            ind = obstacles[0].mouse_near_vertex(mouse_radius)
            if ind >= 0:
                moving_vertex = True
        if event.type == pygame.MOUSEBUTTONUP:
            moving_vertex = False

    if moving_vertex == True:
        obstacles[0].draw_divider(ind)
        obstacles[0].change_vertex((pos[0]/pixels_per_meter, pos[1]/pixels_per_meter), ind)
        
    pygame.display.flip()
    fpsClock.tick(FPS)
pygame.quit()