#libraries
import pygame
import math
import numpy as np

# TODO: Move these to a class
k, b = 0.03, 0.5
w, h = 700, 700
screen = pygame.display.set_mode(([w, h]), pygame.RESIZABLE) # This one should likely go in the camera class

class Shape:
    def __init__(self):
        pass

    def draw(self, camera):
        pass

    def collide(self, player):
        pass

# TODO: Add a color
class Circle(Shape):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def draw(self, camera):
        adjusted_p_x = w/2 + self.x - camera.p[0]
        adjusted_p_y = h/2 + self.y - camera.p[1]
        pygame.draw.circle(screen, 'blue', (adjusted_p_x, adjusted_p_y), self.r, 2)

    def collide(self, player):
        #creates a unit vector pointing from the circle p to the player.p
        dx = player.p[0] - self.x
        dy = player.p[1] - self.y
        distance = math.sqrt((dx)**2 + (dy)**2)
        unit_vector = np.array([dx/distance, dy/distance], dtype = float)

        # TODO: Refactor this into a function
        #player bounces off circle if they collide
        if distance < player.radius + self.r:
            player.p[0] = self.x + unit_vector[0] * (player.radius + self.r)
            player.p[1] = self.y + unit_vector[1] * (player.radius + self.r)
            dot_product = player.v[0] * unit_vector[0] + player.v[1] * unit_vector[1]
            projection = np.array([dot_product * unit_vector[0], dot_product * unit_vector[1]])
            normal_v = - player.bounciness * projection
            tangential_v = player.slipperiness * (player.v - projection)
            player.v = normal_v + tangential_v
            
            #changes rotation of player (not super physically accurate)
            new_dot_product = tangential_v[0] * dy - tangential_v[1] * dx
            if new_dot_product > 0:
                sign = -1
            else:
                sign = 1
            player.omega = sign * np.linalg.norm(player.slipperiness * (player.v - projection))/player.radius

class Line(Shape):
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.circles = [
            Circle(start[0], start[1], 0),
            Circle(end[0], end[1], 0)
        ]

    def draw(self, camera):
        p1 = np.array([w/2 + self.start[0] - camera.p[0], h/2 + self.start[1] - camera.p[1]], dtype = float)
        p2 = np.array([w/2 + self.end[0] - camera.p[0], h/2 + self.end[1] - camera.p[1]], dtype = float)
        pygame.draw.line(screen, "blue", (p1[0], p1[1]), (p2[0], p2[1]), 2)

    def collide(self, player):
        #creates a unit vector pointing from the start of the line to the end
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        distance = math.sqrt(dx**2 + dy**2)
        line_unit_vector = np.array([dx/distance, dy/distance], dtype = float)
        
        # TODO: Refactor this into a function
        #bounces player off line if they collide
        dot_product = (player.p[0] - self.start[0]) * line_unit_vector[0] + (player.p[1] - self.start[1]) * line_unit_vector[1]
        projection = np.array([dot_product * line_unit_vector[0], dot_product * line_unit_vector[1]])
        normal_vector = np.array([(player.p[0] - self.start[0]), (player.p[1] - self.start[1])], dtype = float) - projection
        norm = normal_vector/np.linalg.norm(normal_vector)
        if dot_product > 0 and np.linalg.norm(projection) < distance:
            if np.linalg.norm(normal_vector) < player.radius:
                player.p = [self.start[0] + projection[0] + player.radius * norm[0]/np.linalg.norm(norm), self.start[1] + projection[1] + player.radius * norm[1]/np.linalg.norm(norm)]
                dot_product = player.v[0] * norm[0] + player.v[1] * norm[1]
                projection = np.array([dot_product * norm[0], dot_product * norm[1]])
                normal_v = - player.bounciness * projection
                tangential_v = player.slipperiness * (player.v - projection)
                player.v = normal_v + tangential_v
                
                #changes rotation of player (not super physically accurate)
                new_dot_product = tangential_v[0] * dx + tangential_v[1] * dy
                if new_dot_product > 0: sign = 1
                else: sign = -1
                player.omega = sign * np.linalg.norm(tangential_v)/player.radius

        self.circles[0].collide(player)
        self.circles[1].collide(player)

class Rectangle(Shape):
    # vec1 and vec2 need better names
    def __init__(self, vec1, vec2):
        x1 = vec1[0]
        y1 = vec1[1]
        x2 = vec2[0]
        y2 = vec2[1]
        self.lines = [Line(np.array([x1, y1], dtype = float), np.array([x1, y2], dtype = float)), 
                      Line(np.array([x1, y1], dtype = float), np.array([x2, y1], dtype = float)), 
                      Line(np.array([x1, y2], dtype = float), np.array([x2, y2], dtype = float)), 
                      Line(np.array([x2, y1], dtype = float), np.array([x2, y2], dtype = float))]

    def draw(self, camera):
        for line in self.lines:
            line.draw(camera)

    def collide(self, player):
        for line in self.lines:
            line.collide(player)


class Player:
    # TODO: A lot of these parameters could be moved to constants
    def __init__(self, p: np.array, v: np.array, radius: float, gravity: float, bounciness: float, slipperiness: float, force: np.array, theta: float, omega: float):
        self.p = p
        self.v = v
        self.radius = radius
        self.gravity = gravity
        self.bounciness = bounciness
        self.slipperiness = slipperiness
        self.base_force = force
        self.force = np.array([0, 0], dtype = float)
        self.theta = theta
        self.omega = omega
        

    def draw(self, camera):
        x = self.p[0]
        y = self.p[1]
        r = self.radius
        theta = self.theta
        Line(
            np.array([x - r * math.cos(theta), y - r * math.sin(theta)], dtype = float),
            np.array([x + r * math.cos(theta), y + r * math.sin(theta)], dtype = float)
        ).draw(camera)
        Line(
            np.array([x - r * math.cos(theta + math.pi/2), y - r * math.sin(theta + math.pi/2)], dtype = float),
            np.array([x + r * math.cos(theta + math.pi/2), y + r * math.sin(theta + math.pi/2)], dtype = float)
        ).draw(camera)
        # 'blue'
        Circle(
            x, y, r
        ).draw(camera)

    def update(self):
        self.a = np.array([self.force[0], self.force[1] + self.gravity], dtype = float)
        self.v += self.a
        self.p += self.v
        rotation_b = 0.008
        self.omega += -rotation_b * self.omega
        self.theta += self.omega

    def collide(self, shapes):
        for shape in shapes:
            shape.collide(self)


class Camera:
    BUFFER_X = 500 
    BUFFER_Y = 250
    
    def __init__(self, p: np.array):
        self.a = np.array([0,0], dtype = float)
        self.v = np.array([0,0], dtype = float)
        self.p = p
        self.target = np.array([p[0],p[1]], dtype = float)

    def draw(self):
        # 'red'
        Circle(self.target[0], self.target[1], 10).draw(self)
        # 'black'
        Circle(self.p[0], self.p[1], 20).draw(self)

    def update(self, player: Player):
        #camera moves vertically if player reaches top or bottom camera buffer
        # TODO: Refactor the target calculation into a function
        w, h = pygame.display.get_surface().get_size()
        if abs(player.p[1] - self.p[1]) > h/2 - Camera.BUFFER_Y:
            if player.p[1] > self.p[1]:
                self.target[1] = player.p[1] - (h/2 - Camera.BUFFER_Y)
            else:
                self.target[1] = player.p[1] + (h/2 - Camera.BUFFER_Y)

        #camera moves towards a target infront of the direction the ball is going
        speed = abs(player.v[0])
        u = min(speed/100, 0.35)
        if player.v[0] > 0:
            self.target[0] = player.p[0] + w * u
        else:
            self.target[0] = player.p[0] - w * u
            
        self.a[0] = -b * self.v[0] +  k * (self.target[0] - self.p[0])
        self.v[0] += self.a[0]
        self.p[0] += self.v[0]
        self.a[1] = -2.1 * b * self.v[1] + 35 * k * (self.target[1] - self.p[1])
        self.v[1] += self.a[1]
        self.p[1] += self.v[1]

class Game:
    FPS = 165
    def __init__(self):
        pygame.init()
        self.debug = False
        self.camera = Camera(np.array([w/2, h/2], dtype = float))
        self.player = Player(np.array([500, 500], dtype = float), np.array([1, 1], dtype = float), 100, 0.1, 0.8, 0.996, 0.15, 0, 0)
        self.shapes = [
            Circle(1000, 1000, 500),
            Rectangle(np.array([5, 5], dtype = float), np.array([4000, 1500], dtype = float)),
            Line(np.array([2000, 1000], dtype = float), np.array([3000, 800], dtype = float))
        ]
        self.clock = pygame.time.Clock()

    def stop_requested(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
        return False

    def handle_toggle_debug(self, keys):
        if keys[pygame.K_d]:
            self.debug = not self.debug
    
    def handle_player_input(self, keys):
        force = self.player.base_force

        force_x, force_y = 0, 0
        # TODO: Simplify this logic, having more than 2-3 if/else statements is usually a code smell
        if keys[pygame.K_UP]:# or keys[pygame.K_SPACE]:
            if keys[pygame.K_DOWN]: force_y = 0
            elif keys[pygame.K_LEFT]: force_x, force_y = -force/math.sqrt(2), -force/math.sqrt(2)
            elif keys[pygame.K_RIGHT]: force_x, force_y = force/math.sqrt(2), -force/math.sqrt(2)
            else: force_y = -force
        elif keys[pygame.K_DOWN]:
            if keys[pygame.K_LEFT]: force_x, force_y = -force/math.sqrt(2), force/math.sqrt(2)
            elif keys[pygame.K_RIGHT]: force_x, force_y = force/math.sqrt(2), force/math.sqrt(2)
            else: force_y = force
        if keys[pygame.K_LEFT] and keys[pygame.K_RIGHT]: force_x = 0
        elif keys[pygame.K_LEFT]: force_x = -force
        elif keys[pygame.K_RIGHT]: force_x = force
    
        return np.array([force_x, force_y], dtype = float)
    
    def handle_input(self):
        keys = pygame.key.get_pressed()
        self.handle_toggle_debug(keys)
        self.player.force = self.handle_player_input(keys)

    def update(self):
        self.player.update()
        self.camera.update(self.player)
        self.player.collide(self.shapes)

    def draw(self):
        screen.fill("white")
        self.player.draw(self.camera)
        if self.debug:
            self.camera.draw()
        for shape in self.shapes:
            shape.draw(self.camera)
        pygame.display.flip()

    def run(self):
        while not self.stop_requested():
            self.handle_input()
            self.update()
            self.draw()
            self.clock.tick(Game.FPS)

Game().run()
