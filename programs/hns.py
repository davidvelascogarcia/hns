'''
---------------------------------------------------
Program: Heuristic Navigation System
Type: Python
Author: David Velasco Garcia @davidvelascogarcia
---------------------------------------------------
'''

# Libraries
import argparse
import configparser
import datetime
from halo import Halo
import numpy as np
import pandas as pd
import platform
import pygame
import time
import threading
import queue
import yarp


# Class Heuristic Navigation System
class HNS:

    def __init__(self):

        '''
        Build HNS object.
        Prepare HALO decorator parameters.
        '''

        # Build Halo spinner
        self.halo_response = Halo(spinner='dots')

        # Build default targets
        self.default_init_x = 2
        self.default_init_y = 2
        self.default_goal_x = 7
        self.default_goal_y = 2

        # Build actual coordinates
        self.coordinate_x = self.default_init_x
        self.coordinate_y = self.default_init_y

    # Function: get_platform
    def get_platform(self):

        '''
        Get system platform and release.

        :return: sys_platform: string, sys_release: string
        '''

        # Get system configuration
        print("\nDetecting system and release version ...\n")
        sys_platform = platform.system()
        sys_release = platform.release()

        print("**************************************************************************")
        print("Configuration detected:")
        print("**************************************************************************")
        print("\nPlatform:")
        print(sys_platform)
        print("Release:")
        print(sys_release)

        return sys_platform, sys_release

    # Function: get_config
    def get_config(self):

        '''
        Get config data. Select map dir, map and initial/goal locations.

        :return: map: string, init_x: int, init_y: int, goal_x: int, goal_y: int
        '''

        try:
            # Configure parser arguments
            parser = argparse.ArgumentParser()

            parser.add_argument('--dir', default='./../maps')
            parser.add_argument('--map', default='map11.csv')
            parser.add_argument('--init', default="2,2")
            parser.add_argument('--goal', default="21,19")
            parser.add_argument('--yarp', default=False)
            parser.add_argument('--target', default="/robot/controller:i")
            parser.add_argument('--response', default="/robot/controller:o")

            # Parse input data
            parsed = parser.parse_args()

            dir = parsed.dir
            map = parsed.map
            init = parsed.init
            goal = parsed.goal
            yarp_mode = parsed.yarp
            target = parsed.target
            target_response = parsed.response

            # Prepare full map path
            map = str(dir) + "/" + str(map)

            # Split initial location
            init = init.split(",")
            init_x = init[0]
            init_y = init[1]

            # Split goal location
            goal = goal.split(",")
            goal_x = goal[0]
            goal_y = goal[1]

        except Exception as ex:

            halo_message = "\n[ERROR] Error parsing parameters: " + str(ex) + " ...\n"
            self.halo_response.text_color = "red"
            self.halo_response.fail(halo_message)

            map = "./../maps/map3.csv"
            init_x = self.default_init_x
            init_y = self.default_init_y
            goal_x = self.default_goal_x
            goal_y = self.default_goal_y
            yarp_mode = False

        return map, init_x, init_y, goal_x, goal_y, yarp_mode, target, target_response

    # Function: get_map
    def get_map(self, map):

        '''
        Get map and transform to 2D matrix.

        :param map: string
        :return: map: int
        '''

        # Read CSV map with , delimiter and transform to matrix
        map = np.genfromtxt(map, delimiter=',')
        map = map.astype("i")

        return map

    # Function: assign_targets
    def assign_targets(self, map, init_x, init_y, goal_x, goal_y):

        '''
        Assign init and goal locations.

        :param map: int
        :param init_x: int
        :param init_y: int
        :param goal_x: int
        :param goal_y: int
        :return: map: int
        '''

        result = self.check_location(map, init_x, init_y)

        if result:
            map[int(init_x)][int(init_y)] = 3
            self.update_location(int(init_x), int(init_y))
        else:
            map[int(self.default_init_x)][int(self.default_init_y)] = 3
            self.update_location(int(self.default_init_x), int(self.default_init_y))

        result = self.check_location(map, goal_x, goal_y)

        if result:
            map[int(goal_x)][int(goal_y)] = 4
        else:
            map[int(self.default_goal_x)][int(self.default_goal_y)] = 4

        return map

    # Function: check_location
    def check_location(self, map, x_coordinate, y_coordinate):

        '''
        Check if location is available.

        :param map: int
        :param x_coordinate: int
        :param y_coordinate: int
        :return: result: bool
        '''

        try:
            if map[int(x_coordinate), int(y_coordinate)] == 0:
                result = True

            else:
                halo_message = "\n[ERROR] Error location is not available.\n"
                self.halo_response.text_color = "red"
                self.halo_response.fail(halo_message)

                result = False

        except Exception as ex:
            halo_message = "\n[ERROR] Error location out of the map: " + str(ex) + " ...\n"
            self.halo_response.text_color = "red"
            self.halo_response.fail(halo_message)

            result = False

        return result

    # Function: resume
    def resume(self, init_x, init_y, goal_x, goal_y):

        '''
        Print path planning resume.

        :param init_x: int
        :param init_y: int
        :param goal_x: int
        :param goal_y: int
        :return: None
        '''

        print("**************************************************************************")
        print("Resume:")
        print("**************************************************************************")
        print("\nInit coordinates:")
        print("Coordinate X: " + str(init_x))
        print("Coordinate Y: " + str(init_y))
        print("\nGoal coordinates:")
        print("Coordinate X: " + str(goal_x))
        print("Coordinate Y: " + str(goal_y))

        print("\n**************************************************************************")
        print("Map:")
        print("**************************************************************************")

    # Function: show_map
    def show_map(self, map):

        '''
        Show map with pretty format.

        :param map: int
        :return: None
        '''

        pretty_map = pd.DataFrame(map)
        pretty_map = pretty_map.replace(0, " ")
        pretty_map = pretty_map.replace(1, "||")
        pretty_map = pretty_map.replace(2, ".")
        pretty_map = pretty_map.replace(3, "S")
        pretty_map = pretty_map.replace(4, "E")

        print("\n" + pretty_map.to_string())

    # Function: analyze
    def analyze(self, map, coordinate_x, coordinate_y):

        '''
        Analyze map status.

        :param map: int
        :param coordinate_x: int
        :param coordinate_y: int
        :return: up_status: int, down_status: int, right_status: int, left_status: int
        '''

        # Analyze up position
        try:
            up_status = map[int(coordinate_x) - 1][int(coordinate_y)]

        # If error because it´s out of the map, set as obstacle
        except:
            up_status = '1'

        # Analyze down position
        try:
            down_status = map[int(coordinate_x) + 1][int(coordinate_y)]

        # If error because it´s out of the map, set as obstacle
        except:
            down_status = '1'

        # Analyze right position
        try:
            right_status = map[int(coordinate_x)][int(coordinate_y) + 1]

        # If error because it´s out of the map, set as obstacle
        except:
            right_status = '1'

        # Analyze left position
        try:
            left_status = map[int(coordinate_x)][int(coordinate_y) - 1]

        # If error because it´s out of the map, set as obstacle
        except:
            left_status = '1'

        return up_status, down_status, right_status, left_status

    # Function: update_location
    def update_location(self, coordinate_x, coordinate_y):

        '''
        Update actual coordinates.

        :param coordinate_x: int
        :param coordinate_y: int
        :return:
        '''

        self.coordinate_x = coordinate_x
        self.coordinate_y = coordinate_y

        return coordinate_x, coordinate_y

    # Function: clear_previous_location
    def clear_previous_location(self, map, coordinate_x, coordinate_y):

        '''
        Clear previous location in map.

        :param map: int
        :param coordinate_x: int
        :param coordinate_y: int
        :return: map: int
        '''

        if (map[int(coordinate_x)][int(coordinate_y)] == 3) or (map[int(coordinate_x)][int(coordinate_y)] == 4):
            print("Singular point.")

        else:
            map[int(coordinate_x)][int(coordinate_y)] = 2

        return map

    # Function: update_map
    def update_map(self, map, coordinate_x, coordinate_y):

        '''
        Update map.

        :param map: int
        :param coordinate_x: int
        :param coordinate_y: int
        :return: map: int
        '''

        if (map[int(coordinate_x)][int(coordinate_y)] == 3) or (map[int(coordinate_x)][int(coordinate_y)] == 4):
            print("Singular point.")

        else:
            map[int(coordinate_x)][int(coordinate_y)] = 2

        return map

    # Function: get_distance
    def get_distance(self, coordinate_x, coordinate_y, goal_x, goal_y):

        '''
        Get distance to goal with and without direction.

        :param coordinate_x: int
        :param coordinate_y: int
        :param goal_x: int
        :param goal_y: int
        :return: distance_x: int, distance_y: int, abs_distance_x: int, abs_distance_y: int
        '''

        distance_x = int(goal_x) - int(coordinate_x)
        distance_y = int(goal_y) - int(coordinate_y)

        abs_distance_x = abs(distance_x)
        abs_distance_y = abs(distance_y)

        return distance_x, distance_y, abs_distance_x, abs_distance_y

    # Function: take_decision
    def take_decision(self, up, down, right, left, distance_x, distance_y, abs_distance_x, abs_distance_y, buffer):

        '''
        Take decision. Preference higher distance sense, second higher distance sense, second higher distance invert sense, higher distance invserse sense.

        :param buffer: queue
        :param up: int
        :param down: int
        :param right: int
        :param left: int
        :param distance_x: int
        :param distance_y: int
        :param abs_distance_x: int
        :param abs_distance_y: int
        :return: command: string:, buffer: queue
        '''

        # Init command
        command = "NONE"

        # Priority:
        # If vertical distance is bigger than horizontal try to do vertical move
        if (int(abs_distance_x) >= int(abs_distance_y)):
            # If goal point is up try to go up
            if (int(distance_x) < 0):
                # If space is free or is goal
                if ((up == 0) or (up == 4)):
                    command = "UP"

                # If is an obstacle try to go second priority
                else:
                    # If goal is left
                    if (int(distance_y) < 0):
                        if ((left == 0) or (left == 4)):
                            command = "LEFT"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((right == 0) or (right == 4)):
                                command = "RIGHT"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((down == 0) or (down == 4)):
                                    command = "DOWN"

                    # If goal is right
                    else:
                        if ((right == 0) or (right == 4)):
                            command = "RIGHT"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((left == 0) or (left == 4)):
                                command = "LEFT"
                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((down == 0) or (down == 4)):
                                    command = "DOWN"

            # If goal point is down
            else:
                if ((down == 0) or (down == 4)):
                    command = "DOWN"

                # If is an obstacle try to go second priority
                else:
                    # If goal point is left
                    if (int(distance_y) < 0):
                        if ((left == 0) or (left == 4)):
                            command = "LEFT"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((right == 0) or (right == 4)):
                                command = "RIGHT"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((up == 0) or (up == 4)):
                                    command = "UP"

                    # If goal point is right
                    else:
                        if ((right == 0) or (right == 4)):
                            command = "RIGHT"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((left == 0) or (left == 4)):
                                command = "LEFT"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((up == 0) or (up == 4)):
                                    command = "UP"

        # If horizontal distance is bigger than vertical try to go in horizontal move
        else:
            # If goal point is left
            if (int(distance_y) < 0):
                if ((left == 0) or (left == 4)):
                    command = "LEFT"

                # If is an obstacle try to go second priority
                else:
                    # If goal point is up
                    if (int(distance_x) < 0):
                        if ((up == 0) or (up == 4)):
                            command = "UP"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((down == 0) or (down == 4)):
                                command = "DOWN"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((right == 0) or (right == 4)):
                                    command = "RIGHT"

                    # If goal point is down
                    else:
                        if ((down == 0) or (down == 4)):
                            command = "DOWN"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((up == 0) or (up == 4)):
                                command = "UP"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((right == 0) or (right == 4)):
                                    command = "RIGHT"

            # If goal point is right
            else:
                if ((right == 0) or (right == 4)):
                    command = "RIGHT"

                # If is an obstacle try to go second priority
                else:
                    # If goal point is up
                    if (int(distance_x) < 0):
                        if ((up == 0) or (up == 4)):
                            command = "UP"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((down == 0) or (down == 4)):
                                command = "DOWN"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((left == 0) or (left == 4)):
                                    command = "LEFT"

                    # If goal point is down
                    else:
                        if ((down == 0) or (down == 4)):
                            command = "DOWN"

                        # If second priority it´s not available try to go to third tha is different direction
                        else:
                            if ((up == 0) or (up == 4)):
                                command = "UP"

                            # If third priority it´s not free try to go to the last priority
                            else:
                                if ((right == 0) or (right == 4)):
                                    command = "LEFT"

        # Set command into queue buffer
        buffer.put(command)

    # Function: move
    def move(self, map, command, coordinate_x, coordinate_y):

        '''
        Move system with decision planned.

        :param map: int
        :param command: string
        :param coordinate_x: int
        :param coordinate_y: int
        :return: map: int
        '''

        map = self.clear_previous_location(map, coordinate_x, coordinate_y)

        if str(command) == "UP":
            coordinate_x = int(coordinate_x) - 1
            coordinate_y = int(coordinate_y)

        if str(command) == "DOWN":
            coordinate_x = int(coordinate_x) + 1
            coordinate_y = int(coordinate_y)

        if str(command) == "RIGHT":
            coordinate_x = int(coordinate_x)
            coordinate_y = int(coordinate_y) + 1

        if str(command) == "LEFT":
            coordinate_x = int(coordinate_x)
            coordinate_y = int(coordinate_y) - 1

        coordinate_x, coordinate_y = self.update_location(coordinate_x, coordinate_y)
        map = self.update_map(map, coordinate_x, coordinate_y)

        return map, coordinate_x, coordinate_y

    # Function: goal_achieved
    def goal_achieved(self):

        '''
        Goal achived message.

        :return: None
        '''

        halo_message = "\n[INFO] Goal achieved\n"
        self.halo_response.text_color = "green"
        self.halo_response.succeed(halo_message)

    # Function: goal_not_achieved
    def goal_not_achieved(self):

        '''
        Goal not achived message
        :return: None
        '''

        halo_message = "\n[WARN] Goal not achieved, limit steps.\n"
        self.halo_response.text_color = "yellow"
        self.halo_response.warn(halo_message)

    # Function: end_resume
    def end_resume(self, init_time, steps):

        '''
        Print end resume.

        :param init_time: datetime
        :param steps: int
        :return:
        '''

        # Get end time
        end_time = datetime.datetime.now()

        # Compute elapsed time
        elapsed_time = end_time - init_time

        print("\n**************************************************************************")
        print("Resume: ")
        print("**************************************************************************\n")
        print("Init time: " + str(init_time))
        print("End time: " + str(end_time))
        print("Elapsed time: " + str(elapsed_time))
        print("Steps: " + str(steps))

    # Function: yarp_connection
    def yarp_connection(self, hns_output_port, hns_input_port, target, target_response):

        '''
        Connect yarp ports.

        :param hns_output_port: YarpDataPort
        :param hns_input_port: YarpDataPort
        :param input_target: string
        :param output_target: string
        :return: None
        '''

        # Connect yarp ports
        print(hns_output_port.name, target)
        print(target_response, hns_input_port.name)
        yarp.Network.connect(hns_output_port.name, target)
        yarp.Network.connect(target_response, hns_input_port.name)

        print("conectado")

    # Function: yarp_interaction
    def yarp_interaction(self, hns_output_port, hns_input_port, command):

        '''
        Send command to system and wait until action is executed and received callback.

        :param hns_output_port: YarpDataPort
        :param hns_input_port: YarpDataPort
        :param command: string
        :return: None
        '''

        hns_output_port.send(command)
        response = hns_input_port.receive()

        halo_message = "\n[INFO] Response: " + str(response) + "\n"
        self.halo_response.text_color = "green"
        self.halo_response.succeed(halo_message)

    # Function: process
    def process(self, map, init_x, init_y, goal_x, goal_y, yarp_mode, hns_output_port, hns_input_port, screen, screen_width, screen_height, rows, columns, background, wall, robot, sim_period, init):

        '''
        Process path planning.

        :param yarp_mode: bool
        :param hns_output_port: YarpDataPort
        :param hns_input_port: YarpDataPort
        :param map: int
        :param init_x: int
        :param init_y: int
        :param goal_x: int
        :param goal_y: int
        :return: None
        '''

        # Get init time
        init_time = datetime.datetime.now()

        # Update init location
        coordinate_x, coordinate_y = self.update_location(init_x, init_y)

        # Control loop
        goal = False
        steps = 0

        while not goal:

            print("\n**************************************************************************")
            print("Step: " + str(steps))
            print("**************************************************************************\n")

            # Get distances to goal
            distance_x, distance_y, abs_distance_x, abs_distance_y = self.get_distance(coordinate_x, coordinate_y, goal_x, goal_y)

            # Analyze environment
            up, down, right, left = self.analyze(map, coordinate_x, coordinate_y)

            # If goal achieved
            if (int(abs_distance_x) == 0) and (int(abs_distance_y) == 0):
                self.goal_achieved()

                # Exit loop
                goal = True

            # In process
            else:
                # Build queue buffer
                buffer = queue.Queue()

                # Build thread to take decision target
                thread = threading.Thread(target=self.take_decision, args=(up, down, right, left, distance_x, distance_y, abs_distance_x, abs_distance_y, buffer))

                # Start thread
                thread.start()

                # Wait until thread ends
                thread.join()

                # Get take decison command from buffer
                command = buffer.get()

                # Move with selected command
                map, coordinate_x, coordinate_y = self.move(map, command, coordinate_x, coordinate_y)

                halo_message = "\n[INFO] Command: " + str(command) + "\n"
                self.halo_response.text_color = "blue"
                self.halo_response.info(halo_message)

                # If yarp mode, send command and wait callback
                if yarp_mode:
                    self.yarp_interaction(hns_output_port, hns_input_port, command)

                # If limit steps
                if int(steps) > 100:
                    self.goal_not_achieved()

                    # Exit loop
                    goal = True

            # Increase steps
            steps = steps + 1

            # Show map
            self.show_map(map)

            # Update PyGame screen
            init = screen.simulate(screen_width, screen_height, map, rows, columns, background, wall, robot, sim_period, coordinate_y, coordinate_y, init)

        # If yarp mode, send GOAL command and wait callback
        if yarp_mode:
            self.yarp_interaction(hns_output_port, hns_input_port, "GOAL")

        # Print end resume
        self.end_resume(init_time, steps)


# Class: yarp data port
class YarpDataPort:

    '''
    Class yarp data ports. Allow build sender and receiver ports and close them.
    '''

    # Function: Constructor
    def __init__(self, name):

        '''
        Build port object and open.

        :param name: string
        '''

        # Build Halo spinner
        self.halo_response = Halo(spinner='dots')

        # Build port and bottle
        self.yarp_port = yarp.Port()
        self.yarp_bottle = yarp.Bottle()

        halo_message = "\n[INFO] Opening Yarp data port " + str(name) + " ...\n"
        self.halo_response.text_color = "yellow"
        self.halo_response.warn(halo_message)

        # Open yarp port
        self.name = name
        self.yarp_port.open(self.name)

    # Function: receive
    def receive(self):

        '''
        Receive data from yarp port and return it.

        :return: data: string
        '''

        self.yarp_port.read(self.yarp_bottle)
        data = self.yarp_bottle.toString()
        data = data.replace('"', '')

        halo_message = "\n[RECEIVED] Data received: " + str(data) + " at " + str(datetime.datetime.now()) + ".\n"
        self.halo_response.text_color = "blue"
        self.halo_response.info(halo_message)

        return data

    # Function: send
    def send(self, data):

        '''
        Send data with yarp port.

        :param data: string
        :return: None
        '''

        self.yarp_bottle.clear()
        self.yarp_bottle.addString(str(data))
        self.yarp_port.write(self.yarp_bottle)

    # Function: close
    def close(self):

        '''
        Close yarp port.

        :return: None
        '''

        halo_message = "\n[INFO] " + str(self.name) + " port closed correctly.\n"
        self.halo_response.text_color = "yellow"
        self.halo_response.warn(halo_message)

        self.yarp_port.close()


# Class: Screen
class Screen:

    '''
    Class PyGame simulation screen.
    '''

    # Function: Constructor
    def __init__(self):

        '''
        Build PyGame screen.
        '''

        self.pygame_screen = "NULL"

        # Build Halo spinner
        self.halo_response = Halo(spinner='dots')

        self.icon = pygame.image.load('./../resources/icon.png')
        self.robot_icon = pygame.image.load('./../resources/robot.png')
        self.start_icon = pygame.image.load('./../resources/start.png')
        self.goal_icon = pygame.image.load('./../resources/goal.png')

    # Function: config
    def config(self):

        '''
        Get simulation parameters like screen size, colors and simulation time.

        :return: screen_width: int, screen_height: int, background: int, wall: int, robot: int
        '''

        # Control read from file
        read = True

        while read:
            try:
                # Get config data
                print("\nGetting config data ...\n")

                configurationObject = configparser.ConfigParser()
                configurationObject.read('../config/config.ini')
                configurationObject.sections()

                screen_width = configurationObject['Configuration']['screen-width']
                screen_height = configurationObject['Configuration']['screen-height']

                background_color_r = configurationObject['Background']['red']
                background_color_g = configurationObject['Background']['green']
                background_color_b = configurationObject['Background']['blue']

                wall_color_r = configurationObject['Wall']['red']
                wall_color_g = configurationObject['Wall']['green']
                wall_color_b = configurationObject['Wall']['blue']

                robot_color_r = configurationObject['Robot']['red']
                robot_color_g = configurationObject['Robot']['green']
                robot_color_b = configurationObject['Robot']['blue']

                sim_period = configurationObject['Period']['sim-period']

                # Build color vectors
                background = (int(background_color_r), int(background_color_g), int(background_color_b))
                wall = (int(wall_color_r), int(wall_color_g), int(wall_color_b))
                robot = (int(robot_color_r), int(robot_color_g), int(robot_color_b))

                # Convert sim period to float
                sim_period = float(sim_period)

                # Exit loop
                read = False

            except:
                halo_message = "\n[ERROR] Sorry, config.ini not founded, waiting 4 seconds to the next check ...\n"
                self.halo_response.text_color = "red"
                self.halo_response.fail(halo_message)
                time.sleep(4)

        halo_message = "\n[INFO] Data obtained correctly.\n"
        self.halo_response.text_color = "green"
        self.halo_response.succeed(halo_message)

        return screen_width, screen_height, background, wall, robot, sim_period

    # Function: get_map_size
    def get_map_size(self, map):

        '''
        Get map parameters. Number of rows and columns.

        :param map: int
        :return: rows: int, columns: int
        '''

        rows = map.shape[0]
        columns = map.shape[1]

        return rows, columns

    # Function: build_screen
    def build_screen(self, screen_width, screen_height):

        '''
        Build PyGame screen.

        :param screen_height: int
        :param screen_width: int
        :return: None
        '''

        self.pygame_screen = pygame.display.set_mode((int(screen_width), int(screen_height)), 0, 32)
        pygame.display.set_caption("HNS")
        pygame.display.set_icon(self.icon)

    # Function: simulate
    def simulate(self, screen_width, screen_height, map, rows, columns, background, wall, robot, sim_period, coordinate_x, coordinate_y, init):

        '''
        Simulate PyGame screen.

        :param screen_height: int
        :param screen_width: int
        :param map: int
        :param rows: int
        :param columns: int
        :param background: int
        :param wall: int
        :param robot: int
        :param sim_period: float
        :param coordinate_x: int
        :param coordinate_y: int
        :return: init: bool
        '''

        pygame.display.update()
        # Draw all simulation map with background color
        self.pygame_screen.fill(background)

        # Compute pixels
        x_pixel = int(screen_width) / int(columns)
        y_pixel = int(screen_height) / int(rows)

        # For each row in simulation map
        for row in range(rows):
            # For each column in row
            for column in range(columns):

                # If simulation map value is 0, print with wall color
                if map[row][column] == 0:
                    pygame.draw.rect(self.pygame_screen, (200, 200, 200), pygame.Rect(x_pixel * column, y_pixel * row, x_pixel, y_pixel))

                # If simulation map value is 2, is past path
                if map[row][column] == 2:
                    pygame.draw.rect(self.pygame_screen, (38, 166, 240), pygame.Rect(x_pixel * column, y_pixel * row, x_pixel, y_pixel))
                    coordinate_x = column
                    coordinate_y = row

                # If simulation map value is 3, print with start color
                if map[row][column] == 3:
                    pygame.draw.rect(self.pygame_screen, (169, 240, 38), pygame.Rect(x_pixel * column, y_pixel * row, x_pixel, y_pixel))
                    init_x = column
                    init_y = row

                # If simulation map value is 4, print with goal color
                if map[row][column] == 4:
                    pygame.draw.rect(self.pygame_screen, (255, 87, 51), pygame.Rect(x_pixel * column, y_pixel * row, x_pixel, y_pixel))
                    goal_x = column
                    goal_y = row

        # Draw icons
        self.pygame_screen.blit(self.robot_icon, (x_pixel * (int(coordinate_x) + 0.15), y_pixel * (int(coordinate_y) - 0.5), x_pixel, y_pixel))
        self.pygame_screen.blit(self.goal_icon, (x_pixel * (goal_x + 0.25), y_pixel * (goal_y - 0.5)))
        self.pygame_screen.blit(self.start_icon, (x_pixel * (init_x + 0.12), y_pixel * (init_y - 0.5)))

        # If first time
        if not init:
            # Display simulation map first time
            pygame.display.flip()

            # Set init
            init = True
        else:
            # Update display
            pygame.display.update()

        time.sleep(sim_period / 1000.0)

        return init


# Function: main
def main():

    '''
    Process HNS.

    :return: None
    '''

    print("**************************************************************************")
    print("**************************************************************************")
    print("                   Program: Heuristic Navigation System                   ")
    print("                     Author: David Velasco Garcia                         ")
    print("                             @davidvelascogarcia                          ")
    print("**************************************************************************")
    print("**************************************************************************")

    print("\nLoading hns engine ...\n")

    # Build HNS
    hns = HNS()

    # Get platform
    sys_platform, sys_release = hns.get_platform()

    # Get config
    map, init_x, init_y, goal_x, goal_y, yarp_mode, target, target_response = hns.get_config()

    # If yarp mode is True, init network, build ports and connect to targets
    if yarp_mode:
        yarp.Network.init()
        hns_output_port = YarpDataPort("/hns/controller:o")
        hns_input_port = YarpDataPort("/hns/controller:i")
        hns.yarp_connection(hns_output_port, hns_input_port, target, target_response)

    else:
        hns_output_port = "NULL"
        hns_input_port = "NULL"

    # Get map
    map = hns.get_map(map)

    # Assign targets
    map = hns.assign_targets(map, init_x, init_y, goal_x, goal_y)

    # Print resume
    hns.resume(init_x, init_y, goal_x, goal_y)

    # Show map
    hns.show_map(map)

    # Build Screen
    screen = Screen()

    # Get simulation config parameters
    screen_width, screen_height, background, wall, robot, sim_period = screen.config()

    # Get number of rows and columns
    rows, columns = screen.get_map_size(map)

    # Config build screen
    screen.build_screen(screen_width, screen_height)

    # Simulate screen
    init = screen.simulate(screen_width, screen_height, map, rows, columns, background, wall, robot, sim_period, init_x,
             init_y, False)

    # Process path planning
    hns.process(map, init_x, init_y, goal_x, goal_y, yarp_mode, hns_output_port, hns_input_port, screen, screen_width, screen_height, rows, columns, background, wall, robot, sim_period, init)

    # Close yarp ports
    if yarp_mode:
        hns_output_port.close()
        hns_input_port.close()

    print("\n**************************************************************************")
    print("Program finished")
    print("**************************************************************************")
    print("\nhns program finished correctly.\n")

    exit_program = input()


# Check main module and not a package
if __name__ == "__main__":

    '''
    Calls main module
    '''

    main()

