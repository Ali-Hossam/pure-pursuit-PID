"""Note that this algorithm is not working properly, there is something wrong
in interpoation function, and in updating angles and speeds, i couldn't really
find out what is wrong"""

import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import time
from shapely.geometry import LineString
from shapely.geometry import Point


# Pure pursuit parameters
ld = 1.4  # look-ahead distance
lfg = 0.2  # Look forward gain
wheelbase = 0.8  # [m] wheel base of vehicle

# PID parameters
_kp = 1.1  # speed proportional gain
_kd = 0.15
_ki = 0.1
_dt = 0.1  # time tick


class State:
    """This class is used to store every point state, like x and y coordinates,
    velocity, and rotation angle"""

    def __init__(self, x, y, v, dt=_dt, theta=0, steering_angle=0):
        self.x = x
        self.y = y
        self.v = v
        self.dt = _dt
        self.theta = theta
        self.steering_angle = steering_angle
        self.rear_x = self.x - (wheelbase * math.cos(self.theta))
        self.rear_y = self.y - (wheelbase * math.sin(self.theta))

    def update(self, steering_angle, a):
        self.x = self.x + self.v * math.cos(self.theta) * self.dt
        self.y = self.y + self.v * math.sin(self.theta) * self.dt
        self.v = self.v + a * self.dt
        self.theta = self.theta + \
            (self.v * math.tan(steering_angle) / wheelbase) * self.dt
        self.steering_angle = self.steering_angle + steering_angle * self.dt
        self.rear_x = self.x - (wheelbase * math.cos(self.theta))
        self.rear_y = self.y - (wheelbase * math.sin(self.theta))


class Final_Data:
    """This class is used to store the data for visualization"""

    def __init__(self, state):
        self.state = state
        self.x = []
        self.y = []
        self.theta = []
        self.v = []

    def update(self):
        self.x.append(self.state.x)
        self.y.append(self.state.y)
        self.v.append(self.state.v)
        self.theta.append(self.state.theta)


def remove_dataframe_row(dataframe):
    """removes row 0 from the dataframe"""
    dataframe.drop(0, inplace=True)
    dataframe.reset_index(inplace=True, drop=True)
    return dataframe


def interpolate(point1, point2, radius, center):
    """interpolates between two points one outside look forward ahead and the 
    other is inside, and get a point on the look forward head boundary and on
    the line connects between those points"""
    if point1.y - point2.y == 0:
        new_x = center.rear_x + \
            np.sqrt(radius**2 - (point1.y - center.rear_y)**2)
        new_y = point1.y

    elif point1.x - point2.x == 0:
        new_y = center.rear_y + \
            np.sqrt(radius**2 - (point1.x - center.rear_x)**2)
        new_x = point1.x

    else:
        p = Point([center.rear_x, center.rear_y])
        c = p.buffer(radius).boundary
        l = LineString([[point1.x, point1.y], [point2.x, point2.y]])
        i = c.intersection(l)
        return i.x, i.y
    return new_x, new_y


def calculate_distance_vehicle(point1, vehicle_state):
    """calculate distance between some point and vehicle rear axle point"""
    dx = (point1.x - vehicle_state.rear_x) ** 2
    dy = (point1.y - vehicle_state.rear_y) ** 2
    distance = np.hypot(dx, dy)
    return distance

def pid(target_speed, current_speed, prev_error, integral, dt, kd, kp, ki):
    """apply acceleration based on the difference between my speed and target
    speed"""
    error = target_speed - current_speed
    integral = integral + error * dt
    derivative = (error - prev_error) / dt
    output = (kp * error) + (ki * integral) + (kd * derivative)
    prev_error = error
    time.sleep(dt)
    
    return output, prev_error, integral

def pure_pursuit_PID(path_points_df, ld=ld, lfg=lfg,
                 kp=_kp, kd=_kd, ki=_ki, dt=_dt):
    """Implementation of PurePursuit + PID algorithm
        Args:
            path_points_df: a data frame containing x, y, and required velocity
            at each data point.
            
            ld: look ahead distance
            lfg: look forward gain
            kp, kd, ki, dt: PID parameters
            
        returns x, y, v, and theta at each point in Final data class form
    """
    x, y, v = path_points_df.iloc[0]
    remove_dataframe_row(path_points_df)
    vehicle_state = State(x, y, v)
    final_data = Final_Data(vehicle_state)
    final_data.update()

    prev_error = 0
    integeral = 0

    while len(path_points_df):
        x, y, v = path_points_df.iloc[0]
        point1 = State(x, y, v)
        distance = calculate_distance_vehicle(point1, vehicle_state)
        remove_dataframe_row(path_points_df)

        if distance >= ld:
            next_point = point1
        elif len(path_points_df):
            for i in range(len(path_points_df)):
                x_1, y_1, v_1 = path_points_df.iloc[0]
                point2 = State(x_1, y_1, v_1)

                if calculate_distance_vehicle(point2, vehicle_state) > ld:
                    new_x, new_y = interpolate(
                        point1, point2, ld, vehicle_state)
                    new_v = point2.v
                    next_point = State(new_x, new_y, new_v)
                    remove_dataframe_row(path_points_df)
                    break
                point1 = point2
                remove_dataframe_row(path_points_df)

        theta = math.atan2(
            next_point.y - vehicle_state.y, next_point.x - vehicle_state.x) - vehicle_state.theta

        steering_angle = math.atan(
            (2*ld*math.sin(theta)) / (ld + next_point.v * lfg))
        
        a, prev_error, integeral = pid(next_point.v, vehicle_state.v, prev_error, integeral, 
                                       dt, kd, kp, ki)
        
        vehicle_state.update(steering_angle, a)
        final_data.update()
    return final_data



