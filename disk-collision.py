# a2 q3
# using walls at position x = 500, x = 0, y = 0, and y = 500
import pygame, sys
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import ode
import random

# set up the colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

def normalize(v):
    return v / np.linalg.norm(v)

class Disk2D(pygame.sprite.Sprite):

    def __init__(self, imgfile, radius, mass=1.0):
        pygame.sprite.Sprite.__init__(self)

        self.image = pygame.image.load(imgfile)
        self.image = pygame.transform.scale(self.image, (radius*2, radius*2))
        self.state = [0, 0, 0, 0]
        self.mass = mass
        self.t = 0
        self.radius = radius

        self.solver = ode(self.f)
        self.solver.set_integrator('dop853')
        self.solver.set_initial_value(self.state, self.t)

    def f(self, t, y):
        return [y[2], y[3], 0, 0]

    def set_pos(self, pos):
        self.state[0:2] = pos
        self.solver.set_initial_value(self.state, self.t)
        return self

    def set_vel(self, vel):
        self.state[2:] = vel
        self.solver.set_initial_value(self.state, self.t)
        return self

    def update(self, dt):
        self.t += dt
        self.state = self.solver.integrate(self.t)

    def move_by(self, delta):
        self.state[0:2] = np.add(self.pos, delta)
        return self

    def draw(self, surface):
        rect = self.image.get_rect()
        rect.center = (self.state[0], 640-self.state[1]) # Flipping y
        surface.blit(self.image, rect)

    def pprint(self):
        print 'Disk', self.state

class World:

    def __init__(self):
        self.disks = []
        self.e = 1. # Coefficient of restitution

    def add(self, imgfile, radius, mass=1.0):
        disk = Disk2D(imgfile, radius, mass)
        self.disks.append(disk)
        return disk

    def pprint(self):
        print '#disks', len(self.disks)
        for d in self.disks:
            d.pprint()

    def draw(self, screen):
        for d in self.disks:
            d.draw(screen)

    def update(self, dt):
        self.check_for_collision()
        self.wall_collision()

        for d in self.disks:
            d.update(dt)

    #handle collisions to walls
    def wall_collision(self):
        for i in range (len(self.disks)):
            #if a disk hits the left or right walls
            if self.disks[i].state[0] + self.disks[i].radius > 500 or self.disks[i].state[0] - self.disks[i].radius < 0:
                self.disks[i].set_vel([-self.disks[i].state[2],self.disks[i].state[3]])
            #if a disk hits the upper or bottom walls
            if self.disks[i].state[1] + self.disks[i].radius > 500 or self.disks[i].state[1] - self.disks[i].radius < 0:
                self.disks[i].set_vel([self.disks[i].state[2],-self.disks[i].state[3]])

    #handles collisions between disks
    def check_for_collision(self):
        for i in range (len(self.disks)-1):

            for j in range ((i+1), len(self.disks)):
                if i == j:
                    continue
                #print 'Checking disks', i, 'and', j
                pos_i = np.array(self.disks[i].state[0:2])
                pos_j = np.array(self.disks[j].state[0:2])
                #get distance between the center of the 2 disks
                dist_ij = np.sqrt(np.sum((pos_i - pos_j)**2))

                #print pos_i, pos_j, dist_ij

                radius_i = self.disks[i].radius
                radius_j = self.disks[j].radius
                #if the distance between the 2 disks < the sum of their radius
                if dist_ij > radius_i + radius_j:
                    continue

                # May be a collision (touching each other)
                vel_i = np.array(self.disks[i].state[2:])
                vel_j = np.array(self.disks[j].state[2:])
                relative_vel_ij = vel_i - vel_j
                n_ij = normalize(pos_i - pos_j)

                print relative_vel_ij, n_ij

                if np.dot(relative_vel_ij, n_ij) >= 0:
                    continue

                # collision
                print 'Collision between disks', i, 'and', j, '!!'
                mass_i = self.disks[i].mass
                mass_j = self.disks[j].mass

                # get impulse
                J = -(1+self.e) * np.dot(relative_vel_ij, n_ij) / ((1./mass_i) + (1./mass_j))

                # get new velocities after the collision
                vel_i_aftercollision = vel_i + n_ij * J / mass_i
                vel_j_aftercollision = vel_j - n_ij * J / mass_j

                self.disks[i].set_vel(vel_i_aftercollision)
                self.disks[j].set_vel(vel_j_aftercollision)
                break # Only handle a single collision per instance


def main():

   # initializing pygame
    pygame.init()
    random.seed(0)

    clock = pygame.time.Clock()

    # top left corner is (0,0)
    win_width = 640
    win_height = 640
    screen = pygame.display.set_mode((win_width, win_height))
    screen.fill(WHITE)
    pygame.display.set_caption('Disk-Disk collisions')

    world = World()

    #set the number of disks
    num_disks = 10

    for i in range (num_disks):

        #radius becomes too small to see the disks. defaulted to between 1-2
        #radius=random.uniform(0.1,0.2)
        radius=random.randint(1,5)

        x=random.uniform(radius,500-radius)
        y=random.uniform(radius,500-radius)
        vx=random.uniform(-10,10)
        vy=random.uniform(-10,10)
        mass=random.uniform(1, 5)

        if (i % 3 == 0):
            world.add('disk-blue.png', radius, mass).set_pos([x,y]).set_vel([vx,vy])
        elif(i % 3 == 1):
            world.add('disk-red.png', radius, mass).set_pos([x,y]).set_vel([vx,vy])
        else:
            world.add('disk-pink.png', radius, mass).set_pos([x,y]).set_vel([vx,vy])

    dt = 0.1

    while True:
        # 30 fps
        clock.tick(30)

        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            pygame.quit()
            sys.exit(0)
        else:
            pass

        # Clear the background, and draw the sprites
        screen.fill(WHITE)
        world.draw(screen)

        #draw the walls
        pygame.draw.line(screen, BLACK, (0,win_height-500),(500,win_height-500),1);
        pygame.draw.line(screen, BLACK, (500,win_height-500),(500,win_height),1);

        world.update(dt)

        pygame.display.update()

if __name__ == '__main__':
    main()
