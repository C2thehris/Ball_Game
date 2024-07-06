#comments
'''unused code'''

#libraries
import pygame
import math
import sys
import numpy as np

#initialize game
pygame.init() 
run = True
w, h = 700, 700
screen = pygame.display.set_mode(([w, h]), pygame.RESIZABLE)
FPS = 165 
fpsClock = pygame.time.Clock()
debug_mode = False

#player variables
position = np.array([500, 500], dtype = float)
velocity = np.array([1, 1], dtype = float)
radius = 100
gravity = 0.1
bounciness = 0.8 #1 = large bounce, 0 = no bounce
slipperiness = 0.996 #1 = slippery, 0 = rough
force = 0.15 
theta = 0
omega = 0

#camera variables
camera_buffer_x, camera_buffer_y = 500, 250
camera_velocity_x = 0
camera_position_x = w/2
camera_velocity_y = 0
camera_position_y = h/2
target_x, target_y = w/2, h/2
k, b = 0.03, 0.5

#functions: draw_circle, circle, draw line, line, rectangle
#draws circle on screen relative to camera position
def draw_circle(circle_x, circle_y, circle_radius, color):
    global camera_position_x
    global camera_position_y
    adjusted_position_x = w/2 + circle_x - camera_position_x
    adjusted_position_y = h/2 + circle_y - camera_position_y 
    pygame.draw.circle(screen, color, (adjusted_position_x, adjusted_position_y), circle_radius, 2)

#constructs and draws circle that has collision with the player
def circle(circle_x, circle_y, circle_radius):
    global radius
    global position
    global velocity
    global omega

    #creates a unit vector pointing from the circle position to the player position
    x_displacement = position[0] - circle_x
    y_displacement = position[1] - circle_y
    distance = math.sqrt((x_displacement)**2 + (y_displacement)**2)
    unit_vector = [x_displacement/distance, y_displacement/distance]

    #player bounces off circle if they collide
    if distance < radius + circle_radius:
        position[0], position[1] = circle_x + unit_vector[0] * (radius + circle_radius), circle_y + unit_vector[1] * (radius + circle_radius)
        dot_product = velocity[0] * unit_vector[0] + velocity[1] * unit_vector[1]
        projection = np.array([dot_product * unit_vector[0], dot_product * unit_vector[1]])
        normal_velocity = - bounciness * projection
        tangential_velocity = slipperiness * (velocity - projection)
        velocity = normal_velocity + tangential_velocity
        
        #changes rotation of player (not super physically accurate)
        new_dot_product = tangential_velocity[0] * y_displacement - tangential_velocity[1] * x_displacement
        if new_dot_product > 0: sign = -1
        else: sign = 1
        omega = sign * np.linalg.norm(slipperiness * (velocity - projection))/radius

    #draw circle   
    if circle_radius > 0:
        draw_circle(circle_x, circle_y, circle_radius, 'blue')

#draws line on screen relative to camera position
def draw_line(x1, y1, x2, y2):
    global camera_position_x
    global camera_position_y
    adjusted_x1 = w/2 + x1 - camera_position_x
    adusted_y1 = h/2 + y1 - camera_position_y 
    adjusted_x2 = w/2 + x2 - camera_position_x
    adjusted_y2 = h/2 + y2 - camera_position_y 
    pygame.draw.line(screen, "blue", (adjusted_x1, adusted_y1), (adjusted_x2, adjusted_y2), 2)

#constructs and draws line that has collision with the player
def line(x1, y1, x2, y2):
    global position
    global velocity
    global radius
    global omega

    #creates a unit vector pointing from the start of the line to the end
    x_displacement = x2 - x1
    y_displacement = y2 - y1
    distance = math.sqrt(x_displacement**2 + y_displacement**2)
    line_unit_vector = np.array([x_displacement/distance, y_displacement/distance], dtype = float)
    
    #bounces player off line if they collide
    dot_product = (position[0] - x1) * line_unit_vector[0] + (position[1] - y1) * line_unit_vector[1]
    projection = np.array([dot_product * line_unit_vector[0], dot_product * line_unit_vector[1]])
    normal_vector = np.array([(position[0] - x1), (position[1] - y1)], dtype = float) - projection
    norm = normal_vector/np.linalg.norm(normal_vector)   
    if dot_product > 0 and np.linalg.norm(projection) < distance:
        if np.linalg.norm(normal_vector) < radius:
            position = [x1 + projection[0] + radius * norm[0]/np.linalg.norm(norm), y1 + projection[1] + radius * norm[1]/np.linalg.norm(norm)]
            dot_product = velocity[0] * norm[0] + velocity[1] * norm[1]
            projection = np.array([dot_product * norm[0], dot_product * norm[1]])
            normal_velocity = - bounciness * projection
            tangential_velocity = slipperiness * (velocity - projection)
            velocity = normal_velocity + tangential_velocity
            
            #changes rotation of player (not super physically accurate)
            new_dot_product = tangential_velocity[0] * x_displacement + tangential_velocity[1] * y_displacement
            if new_dot_product > 0: sign = 1
            else: sign = -1
            omega = sign * np.linalg.norm(tangential_velocity)/radius
    
    #add 0 radius circles so player can collide with the ends of the line, and then draw the line 
    circle(x1, y1, 0)
    circle(x2, y2, 0)
    draw_line(x1, y1, x2, y2)

#constucts a rectangle
def rectangle(x1, y1, x2, y2):
    line(x1, y1, x1, y2)
    line(x1, y1, x2, y1)
    line(x1, y2, x2, y2)
    line(x2, y1, x2, y2)

#main chunk of code
while run:
    #Use arrow keys to extert force on the player
    force_x, force_y = 0, 0
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:# or keys[pygame.K_SPACE]:
        if keys[pygame.K_DOWN]: force_y = 0
        elif keys[pygame.K_LEFT]: force_x, force_y = -force/math.sqrt(2), -force/math.sqrt(2)
        elif keys[pygame.K_RIGHT]: force_x, force_y = force/math.sqrt(2), -force/math.sqrt(2)
        else: force_y = -force
    elif keys[pygame.K_DOWN]:
        if keys[pygame.K_LEFT]: force_x, force_y = -force/math.sqrt(2), force/math.sqrt(2)
        elif keys[pygame.K_RIGHT]: force_x, rocket_force_y = force/math.sqrt(2), force/math.sqrt(2)
        else: force_y = force
    if keys[pygame.K_LEFT] and keys[pygame.K_RIGHT]: force_x = 0
    elif keys[pygame.K_LEFT]: force_x = -force
    elif keys[pygame.K_RIGHT]: force_x = force

    #use d key to toggle debug mode
    if keys[pygame.K_d]:
        if temp:
            if debug_mode == True: debug_mode = False
            else: debug_mode = True
            temp = False
    else: temp = True

    #updates player position (acceleration px/frame^2, velocity px/frame, position px)
    acceleration = np.array([force_x, force_y + gravity], dtype = float)
    velocity += acceleration
    position += velocity

    #camera moves vertically if player reaches top or bottom camera buffer
    w, h = pygame.display.get_surface().get_size()
    if abs(position[1] - camera_position_y) > h/2 - camera_buffer_y:
        if position[1] > camera_position_y: target_y = position[1] - (h/2 - camera_buffer_y)
        else: target_y = position[1] + (h/2 - camera_buffer_y)

    #camera moves towards a target infront of the direction the ball is going
    speed = abs(velocity[0])
    u = min(speed/100, 0.35)
    if velocity[0] > 0: target_x = position[0] + w * u
    else: target_x = position[0] - w * u
        
    # camera moves to smoothly follow the player     
    camera_acceleration_x = -b * camera_velocity_x +  k * (target_x - camera_position_x)
    camera_velocity_x += camera_acceleration_x
    camera_position_x += camera_velocity_x
    camera_acceleration_y = -2.1 * b * camera_velocity_y + 35 * k * (target_y - camera_position_y)
    camera_velocity_y += camera_acceleration_y
    camera_position_y += camera_velocity_y

    #used to rotate the ball (not super pysically accurate)
    rotation_b = 0.008
    omega += -rotation_b * omega
    theta += omega

    #reset the screen and draws the player
    screen.fill("white")
    draw_line(position[0] - radius * math.cos(theta), position[1] - radius * math.sin(theta), position[0] + radius * math.cos(theta), position[1] + radius * math.sin(theta))
    draw_line(position[0] - radius * math.cos(theta + math.pi/2), position[1] - radius * math.sin(theta + math.pi/2), position[0] + radius * math.cos(theta + math.pi/2), position[1] + radius * math.sin(theta + math.pi/2))
    draw_circle(position[0], position[1], radius, 'blue')
    
    #create obstacles 
    rectangle(5, 5, 4000, 1500)
    circle(1000, 1000, 500)
    line(2000, 1000, 3000, 800)
    
    #draws camera and camera target for debug mode
    if debug_mode:
        draw_circle(target_x, target_y, 10, 'red')
        draw_circle(camera_position_x, camera_position_y, 20, 'black')

    #quit game
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    pygame.display.flip()
    fpsClock.tick(FPS)
pygame.quit()