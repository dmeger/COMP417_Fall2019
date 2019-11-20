__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

from helpers import *
import sim_env

import random
import cv2
import math
import numpy as np
import csv
import os
import pygame


class Fan:
    def __init__(self, env):
        self.env = env
        self.radius = 0.02
        self.speed = 0.0
        self.force = 0
        self.pitch = 0.2
        self.rpm = 0.0
        self.max_rpm = 100000  # unrealistic but for validation

    def set_rpm(self, rpm):
        if rpm > self.max_rpm:
            rpm = self.max_rpm
        if rpm < 0:
            rpm = 0
        self.rpm = rpm
        rps = rpm / 60.0
        air_velocity = rps * self.pitch
        rho = self.env.mass_density_of_air
        A = math.pi * self.radius**2
        self.force = 0.5 * rho * A * air_velocity**2


class Ball:
    def __init__(self, radius, mass, x, y, env):
        self.use_images = True
        self.preload_images = True
        self.csv_file = 'pics.csv'
        self.images_dir = 'pid_pics'
        self.opencv_pickle_file = os.path.join(self.images_dir, 'opencv_pid_pics.pickle')
        self.keys_pickle_file = os.path.join(self.images_dir, 'keys_pid_pics.pickle')

        self.env = env
        self.pos = [x, y]
        self.vel = [0.0, 0.0]
        self.acc = [0.0, 0.0]
        self.radius = radius
        self.colour = (255, 0, 0)
        self.thickness = 0
        self.speed = 0.0
        self.mass = mass
        self.current_frame = None

        self.opencv_images = {}
        self.keys = []

        self.image_noise = 0.0005

        self.ball_lost_height = 2000
        self.ball_lost = False

        if self.use_images:
            self.load_images()

        pygame.font.init()
        self.font = pygame.font.Font('resources/COMIC.TTF', 25)

    def load_images(self):
        # load the images
        temp_image_paths = {}
        with open(os.path.join(self.images_dir, self.csv_file), 'rb') as csvfile:
            image_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in image_reader:
                temp_image_paths[int(row[0])] = row[1]

        sorted_keys = []
        for key in sorted(temp_image_paths.keys()):
            sorted_keys.append(key)
        max_key = np.max(sorted_keys)

        self.image_paths = {}  # scaled

        for i in sorted_keys:
            scaled = float(i) / (max_key - 1) #this make it just over 1
            self.image_paths[scaled] = temp_image_paths[i]
            if self.preload_images:
                self.opencv_images[scaled] = cv2.imread(os.path.join(self.images_dir, temp_image_paths[i]))
            self.keys.append(scaled)
        # to get the nearest image, use:
        # image_paths[find_nearest(keys, scaled_value)]
        return

    def get_vel_angle(self):
        vel_x = self.vel[0]
        vel_y = self.vel[1]
        angle = 0.5 * math.pi - math.atan2(vel_y, vel_x)
        return angle

    def set_vel(self, angle, vel):
        self.vel[0] = vel * math.sin(angle)
        self.vel[1] * math.cos(angle)

    def reset(self, x, y):
        self.ball_lost = False
        self.pos = [x, y]
        self.vel = [0.0, 0.0]
        self.acc = [0.0, 0.0]
        return

    def update_physics(self, dt, F_external=[0.0, 0.0]):

        prev_pos = list(self.pos)
        prev_vel = list(self.vel)

        # update Forces
        # y direction
        F_g = -1 * self.env.gravity * self.mass
        # F_drag = 0.5 * rho * A * Cd * v^2
        A = math.pi * self.radius ** 2
        rho = self.env.mass_density_of_air
        Cd = self.env.drag_coefficient

        sign = 1.0 if self.vel[1] <= 0.0 else -1.0
        F_drag_y = sign * 0.5 * rho * A * Cd * self.vel[1] ** 2
        F_y = F_g + F_drag_y + F_external[1]
        acc_y = F_y / self.mass
        v0 = prev_vel[1]
        v2 = v0 + acc_y * dt

        new_y_pos = prev_pos[1] + dt * (v2 + v0) / 2.0

        #check if bounced
        if new_y_pos < self.radius:
            ratio = (new_y_pos - self.radius) / (new_y_pos - prev_pos[1])
            v0 = prev_vel[1]
            Fc = sign * 0.5 * rho * A * Cd
            v1 = v0 + (1-ratio) * dt * (F_g + F_external[1] + Fc * v0 * v0) / self.mass
            v2 = -1 * v0 + ratio * dt * (F_g + F_external[1] + -1 * Fc * v1 * -1 * v1) / self.mass
            new_y_pos = self.radius + ratio * dt * (v2 - v1 * self.env.elasticity) / 2.0

        # if new_y_pos > self.env.sim_win_height/self.env.world_scale - self.radius:
        #     ratio = (new_y_pos - (self.env.sim_win_height / self.env.world_scale - self.radius)) / (new_y_pos - prev_pos[1])
        #     # print('bounced: {}'.format(ratio))
        #     v0 = prev_vel[1]
        #     Fc = sign * 0.5 * rho * A * Cd
        #     v1 = v0 + (1-ratio) * dt * (F_g + F_external[1] + Fc * v0 * v0) / self.mass
        #     v2 = -1 * v0 + ratio * dt * (F_g + F_external[1] + -1 * Fc * v1 * -1 * v1) / self.mass
        #     new_y_pos = (self.env.sim_win_height / self.env.world_scale - self.radius) + ratio * dt * (v2 - v1 * self.env.elasticity) / 2.0

        if new_y_pos > self.ball_lost_height:  # the ball will go up but not come back down
            self.ball_lost = True
        else:
            self.vel[1] = v2
            self.pos[1] = new_y_pos
        return

    def display(self):
        if not self.use_images:
            px = self.env.world_scale * self.pos[0]
            py = self.env.sim_win_height - self.env.world_scale * self.pos[1]
            rad = self.env.world_scale * self.radius
            pygame.draw.circle(self.env.screen, self.colour, (int(px), int(py)), int(rad), self.thickness)
        else:
            scaled_height = self.pos[1]

            if self.image_noise > 0:
                scaled_height = self.pos[1] + random.uniform(-self.image_noise, self.image_noise)
            if self.preload_images:
                index = find_nearest(self.keys, scaled_height)
                frame = self.opencv_images[index]
                self.current_frame = frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)
                frame = pygame.surfarray.make_surface(frame)
                self.env.screen.blit(frame, (0, 0))
            else:
                image_path = os.path.join(self.images_dir, self.image_paths[find_nearest(self.keys, scaled_height)])
                frame = cv2.imread(image_path)
                self.current_frame = frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = np.rot90(frame)
                frame = pygame.surfarray.make_surface(frame)
                self.env.screen.blit(frame, (0, 0))
            txt = '{:5.0f} RPM'.format(self.env.fan.rpm)
            self.txt_surf = self.font.render(txt, 1, [255, 255, 255])
            self.txt_rect = self.txt_surf.get_rect(center=(150, 550))
            self.env.screen.blit(self.txt_surf, self.txt_rect)  # this surface never changes
        if self.ball_lost:
            textsurface = self.font.render('Ball LOST!', False, (255, 0, 0))
            self.env.screen.blit(textsurface, (100, self.env.sim_win_height / 2))
