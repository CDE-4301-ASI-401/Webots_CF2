/*
 * Copyright 2022 Bitcraze AB
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file crazyflie_controller.c
 * Description: Controls the crazyflie in webots
 * Author:      Kimberly McGuire (Bitcraze AB)
 */

#include <math.h>
#include <stdio.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Add external controller
#include "pid_controller.h"

// SGBA controller
#include "SGBA.h"
#include <stdlib.h>
#include <unistd.h>
#include <pid_controller.h>


#define FLYING_ALTITUDE 0.3
#define MAX_SPEED 0.3
#define LEFT_WALL_DISTANCE 0.3
#define RIGHT_WALL_DISTANCE 0.7
// #define DESIRED_HEADING_ANGLE -0.5
#define HEADING_INCREMENT 3
#define NUM_OF_DRONES 3

float degrees_to_radians(float degrees) {
    return degrees * (3.14159265358979323846f / 180.0f);
}

int min(int a, int b) {
    return (a < b) ? a : b;
}
// SGBA variables
float vel_x, vel_y, vel_w;
float rssi_angle;
int state_wf;
float range_front_value, range_back_value, range_left_value, range_right_value;

static float heading[3] = { 60.0f, 0.0f, -60.0f };

int main(int argc, char **argv) {
  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  int robot_id = atoi(robot_name);
  printf("Robot id: %d\n", robot_id);
  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, 0.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 0.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, 0.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 0.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);

  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, timestep);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, timestep);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, timestep);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, timestep);

  
  // float desired_angle = (3.14/2) - ((robot_id % HEADING_INCREMENT) * 3.14 / HEADING_INCREMENT );
  float desired_angle = degrees_to_radians(heading[robot_id-1]);

  printf("robot desired angle: %f\n", desired_angle);
  printf("robot_id mod 2 = %d\n", robot_id % 2);
  // float direction = (robot_id % 2 == 1) ? -1 : 1;
  float direction;
  float wall_distance;
  if (robot_id == 1){
    direction = -1;
    wall_distance = LEFT_WALL_DISTANCE;
  }
  else if (robot_id == 2){
    direction = 1;
    wall_distance = RIGHT_WALL_DISTANCE;
  }
  else {
    direction = 1;
    wall_distance = RIGHT_WALL_DISTANCE;
  }

  printf("robot direction: %f\n", direction);
  // Initialize SGBA controller
  if (robot_id % 2 == 1){
    init_SGBA_controller(wall_distance, MAX_SPEED, desired_angle, direction);
  } else {
    init_SGBA_controller(wall_distance, MAX_SPEED, desired_angle, direction);
  }         
  // if (robot_id % 2 == 1){
  //   init_SGBA_controller(LEFT_WALL_DISTANCE, MAX_SPEED, desired_angle, direction);
  // } else {
  //   init_SGBA_controller(RIGHT_WALL_DISTANCE, MAX_SPEED, desired_angle, direction);
  // }       

  // Wait for 2 seconds
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 2.0)
      break;
  }

  // Initialize variables
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  double past_x_global = 0;
  double past_y_global = 0;
  double past_time = wb_robot_get_time();

  // Initialize PID gains.
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 5;
  gains_pid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  double height_desired = FLYING_ALTITUDE;

  // Initialize struct for motor power
  motor_power_t motor_power;

  // printf("\n");

  // printf("====== Controls =======\n");

  // printf(" The Crazyflie can be controlled from your keyboard!\n");
  // printf(" All controllable movement is in body coordinates\n");
  // printf("- Use the up, back, right and left button to move in the horizontal plane\n");
  // printf("- Use Q and E to rotate around yaw\n ");
  // printf("- Use W and S to go up and down\n");

  double get_bearing_in_rad(){
    double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    // printf("Bearing: %f\n", -yaw);
    return yaw;
  }

  while (wb_robot_step(timestep) != -1) {
    // const unsigned char *image = wb_camera_get_image(camera);

    const double dt = wb_robot_get_time() - past_time;
    printf("Time %f\n", wb_robot_get_time());
    // Get measurements
    actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    actual_state.altitude = wb_gps_get_values(gps)[2];
    range_front_value = wb_distance_sensor_get_value(range_front)/1000;
    range_left_value = wb_distance_sensor_get_value(range_left)/1000;
    range_right_value = wb_distance_sensor_get_value(range_right)/1000;
    range_back_value = wb_distance_sensor_get_value(range_back)/1000;
    double x_global = wb_gps_get_values(gps)[0];
    double vx_global = (x_global - past_x_global) / dt;
    double y_global = wb_gps_get_values(gps)[1];
    double vy_global = (y_global - past_y_global) / dt;
    printf("Range front: %f,\tRange Left: %f,\tRange Right: %f, Range back:%f\n",range_front_value, range_left_value, range_right_value, range_back_value);
    // Get body fixed velocities
    double actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actualYaw);
    double sinyaw = sin(actualYaw);
    actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

    // Initialize values
    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = 0;
    desired_state.vy = 0;
    desired_state.yaw_rate = 0;
    desired_state.altitude = 1.0;

    double forward_desired = 0;
    double sideways_desired = 0;
    double yaw_desired = 0;
    double height_diff_desired = 0;

    double heading = get_bearing_in_rad();

    // Control altitude
    // int key = wb_keyboard_get_key();
    // while (key > 0) {
    //   switch (key) {
    //     case WB_KEYBOARD_UP:
    //       forward_desired = +0.5;
    //       break;
    //     case WB_KEYBOARD_DOWN:
    //       forward_desired = -0.5;
    //       break;
    //     case WB_KEYBOARD_RIGHT:
    //       sideways_desired = -0.5;
    //       break;
    //     case WB_KEYBOARD_LEFT:
    //       sideways_desired = +0.5;
    //       break;
    //     case 'Q':
    //       yaw_desired = 1.0;
    //       break;
    //     case 'E':
    //       yaw_desired = -1.0;
    //       break;
    //     case 'W':
    //       height_diff_desired = 0.1;
    //       break;
    //     case 'S':
    //       height_diff_desired = -0.1;
    //       break;
    //   }
    //   key = wb_keyboard_get_key();
    // }
    
    int state = SGBA_controller(&vel_x, &vel_y, &vel_w, &rssi_angle, &state_wf,
                    range_front_value, range_left_value, range_right_value, range_back_value,
                    // current pose
                    heading, x_global, y_global, 
                    // rssi
                    60,60,0.0, 
                    false, true);

    // int state = SGBA_controller(&vel_x, &vel_y, &vel_w, &rssi_angle, &state_wf,
    //                 range_front_value, range_left_value, range_right_value, range_back_value,
    //                 // current pose
    //                 heading, x_global, y_global, 
    //                 // rssi
    //                 60,60,0.0, 
    //                 false, true);
    printf("State = %d, Vel_X = %f, Vel_Y = %f, Vel_W = %f\n", state, vel_x,vel_y,vel_w);
    
    printf("Heading: %f,\tx_pos: %f,\ty_pos: %f,\tz_pos: %f,\t",heading, x_global,y_global,actual_state.altitude);
    height_desired += height_diff_desired * dt;

    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);

    // desired_state.yaw_rate = vel_w;

    // PID velocity controller with fixed height
    // desired_state.vy = vel_y;
    // desired_state.vx = vel_x;
    desired_state.altitude = height_desired;
    printf("Height difff: %f", height_desired-actual_state.altitude);
    ///////////////////////////////////////////////////
    if (height_desired-actual_state.altitude > 0.02){
      printf("Taking off...\n");
      desired_state.yaw_rate = yaw_desired;
      desired_state.vy = sideways_desired;
      desired_state.vx = forward_desired;
      desired_state.altitude = height_desired;
    }
    // else if (vel_w>0.5 || vel_w <-0.5){
    // // else if (vel_w>0.1 || vel_w<-0.1 ) {
    //   printf("Loop 1\n");
    //   desired_state.yaw_rate = vel_w;
    //   // desired_state.vx = vel_x;
    //   // desired_state.vy =  vel_y;
    // }
    ///turn to find wall (does not have vel_w)
    else if ((state_wf==4 && state==3)){
      printf("Loop 2\n");
      desired_state.vx = vel_x;
      desired_state.vy = vel_y;
      desired_state.yaw_rate = direction * MAX_SPEED;   
    }   
    else{
      printf("Loop 3\n");
      desired_state.vx = vel_x;
      desired_state.vy =  vel_y;
      desired_state.yaw_rate = vel_w;
    }
    ///////////////////////////////////////////////////
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // Setting motorspeed
    printf("m1_motor: %f,\t m2_motor: %f,\t m3_motor: %f,\t m4_motor: %f\n\n\n\n", motor_power.m1, motor_power.m2, motor_power.m3, motor_power.m4);
    // motor_power.m1 = min(motor_power.m1,600);
    // motor_power.m2 = min(motor_power.m2,600);
    // motor_power.m3 = min(motor_power.m3,600);
    // motor_power.m4 = min(motor_power.m4,600);
    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    // Save past time for next time step
    past_time = wb_robot_get_time();
    past_x_global = x_global;
    past_y_global = y_global;
    // printf("\n\n");
  };

  wb_robot_cleanup();

  return 0;
}

/***********************************************************
   * SGBA State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = rotate_to_goal
  // 3 = wall_following
  // 4 = move out of way

/***********************************************************
   * WALL-FOLLOWING State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = hover
  // 3 = turn_to_find_wall
  // 4 = turn_to_allign_to_wall
  // 5 = forward along wall
  // 6 = rotate_around_wall
  // 7 = rotate_in_corner
  // 8 = find corner
