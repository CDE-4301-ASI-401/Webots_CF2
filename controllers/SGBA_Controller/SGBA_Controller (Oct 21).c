/*
 * Copyright 1996-2023 Cyberbotics Ltd.
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

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/pen.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
// #include <webots/compass.h>
#include "SGBA.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pid_controller.h>


//#define DEBUG

#define MAX_SPEED 5
#define NUMBER_OF_RANGE_SENSOR 4
#define WALL_DISTANCE 0.1
#define DESIRED_HEADING_ANGLE -0.7
#define HEADING_INCREMENT 4
#define BASE_THRUST 1
#define FLYING_ALTITUDE 1.0

// SGBA variables  
float vel_x, vel_y, vel_w;
float rssi_angle;
int state_wf;
float range_front_value, range_back_value, range_left_value, range_right_value;


int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();
  int i;
  const char *robot_name = wb_robot_get_name();
  int robot_id = atoi(robot_name);
  printf("robot id: %d\n", robot_id);
  float desired_angle = (3.14/2) - ((robot_id % HEADING_INCREMENT) * 3.14 / HEADING_INCREMENT );
  desired_angle = 0;
  printf("robot desired angle: %f\n", desired_angle);
  printf("robot_id mod 2 = %d\n", robot_id % 2);
  float direction = (robot_id % 2 == 0) ? -1 : 1;
  printf("robot direction: %f\n", direction);
  init_SGBA_controller(WALL_DISTANCE, MAX_SPEED, desired_angle, direction);

  // get and enable the camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);

  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, time_step);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, time_step);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, time_step);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, time_step);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);

  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, time_step);


  WbDeviceTag front_right_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(front_right_motor, INFINITY);
  wb_motor_set_velocity(front_right_motor, 10.0);
  WbDeviceTag rear_right_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(rear_right_motor, INFINITY);
  wb_motor_set_velocity(rear_right_motor, 10.0);
  WbDeviceTag rear_left_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(rear_left_motor, INFINITY);
  wb_motor_set_velocity(rear_left_motor, 10.0);
  WbDeviceTag front_left_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(front_left_motor, INFINITY);
  wb_motor_set_velocity(front_left_motor, 10.0);

  // Initialize variables
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  double past_x_global = 0;
  double past_y_global = 0;
  double past_time = wb_robot_get_time();

  // store the last time a message was displayed
  int last_display_second = 0;

  double get_bearing_in_degrees() {
    double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    printf("Yaw : %f\n", yaw);
    // const double *north = wb_compass_get_values(compass);
    // double rad = atan2(north[1], north[0]);
    // double bearing = rad;
    // double bearing = (rad - 1.5708) / M_PI * 180.0;
    // if (bearing < 0.0)
    //   bearing = bearing + 360.0;
    return -yaw;
  }
  // MAIN loop
  while (wb_robot_step(time_step) != -1) {
    const double dt = wb_robot_get_time() - past_time;
    
    // Display some sensor data every second
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second) {
      last_display_second = display_second;

      #ifdef DEBUG
      printf("time = %d [s]\n", display_second);
      #endif
      //for (i = 0; i < 5; ++i)
      //  printf("- ultrasonic sensor('%s') = %f [m]\n", ultrasonic_sensors_names[i],
      //         wb_distance_sensor_get_value(ultrasonic_sensors[i]));
      for (i = 0; i < NUMBER_OF_RANGE_SENSOR; i++)
        //#ifdef DEBUG
        // printf("- Range sensor('%s') = %f [m]\n", range_sensors_names[i],
        //        wb_distance_sensor_get_value(range_sensors_names[i]));
        //     #endif
        printf("Range sensor data placeholder");


    //get gps
    double x_global = wb_gps_get_values(gps)[0];
    double y_global = wb_gps_get_values(gps)[1];

    //get ranges
    range_front_value = wb_distance_sensor_get_value(range_front);
    range_left_value = wb_distance_sensor_get_value(range_left);
    range_right_value = wb_distance_sensor_get_value(range_right);
    range_back_value = wb_distance_sensor_get_value(range_back);
    float heading = (float)get_bearing_in_degrees();
    #ifdef DEBUG
    printf("bearing in rad: %f\n", heading);
    #endif
    // float heading = 0.0;

    int state = SGBA_controller(&vel_x, &vel_y, &vel_w, &rssi_angle, &state_wf,
                    range_front_value, range_left_value, range_right_value, range_back_value,
                    // current pose
                    heading, x_global, y_global, 
                    // rssi
                    60,60,0.0, 
                    false, true);
    
    printf("Vel_X = %f, Vel_Y = %f, Vel_W = %f\n", vel_x,vel_y,vel_w);
    printf("heading, x_global, y_global = %f, %f, %f\n", heading, x_global, y_global);

    //any turning
    if (vel_w>0.1 || vel_w<-0.1 ) {
      printf("here");
      // wb_motor_set_velocity(m1_motor, vel_w);
      // wb_motor_set_velocity(m2_motor, -vel_w);

      // Increase thrust on motors rotating in one direction (front-left and rear-right)
      wb_motor_set_velocity(front_left_motor, BASE_THRUST + vel_w);
      wb_motor_set_velocity(rear_right_motor, BASE_THRUST + vel_w);
      
      // Decrease thrust on opposite motors (front-right and rear-left)
      wb_motor_set_velocity(front_right_motor, BASE_THRUST - vel_w);
      wb_motor_set_velocity(rear_left_motor, BASE_THRUST - vel_w);  
    }
    ///turn to find wall (does not have vel_w)
    else if ((state_wf==4 && state==3)){
      printf("here2");
      // wb_motor_set_velocity(m2_motor, direction * MAX_SPEED);
      // wb_motor_set_velocity(m1_motor, -direction * MAX_SPEED);

      // Increase thrust on motors rotating in one direction (front-left and rear-right)
      wb_motor_set_velocity(front_left_motor, direction * MAX_SPEED);
      wb_motor_set_velocity(rear_right_motor, direction * MAX_SPEED);
      
      // Decrease thrust on opposite motors (front-right and rear-left)
      wb_motor_set_velocity(front_right_motor,-direction * MAX_SPEED);
      wb_motor_set_velocity(rear_left_motor, -direction * MAX_SPEED);     
    }   
    // else if (vel_y>0.1||vel_y<-0.1){
    //   wb_motor_set_velocity(left_motor,  MAX_SPEED-0.05*vel_y);
    //   wb_motor_set_velocity(right_motor, MAX_SPEED+0.05*vel_y);
    } 
    else{
      printf("here3");
      // float motor_speed = 0;
      float motor_speed = MAX_SPEED - 2 * vel_w * MAX_SPEED - 0.05 * vel_y;
      // wb_motor_set_velocity(m1_motor,  MAX_SPEED - 2 * vel_w * MAX_SPEED - 0.05 * vel_y);
      // wb_motor_set_velocity(m2_motor, MAX_SPEED + 2 * vel_w * MAX_SPEED + 0.05 * vel_y);
      // wb_motor_set_velocity(left_motor,  MAX_SPEED);
      // wb_motor_set_velocity(right_motor, MAX_SPEED);

      wb_motor_set_velocity(front_right_motor, motor_speed);
      wb_motor_set_velocity(rear_right_motor, motor_speed);
      wb_motor_set_velocity(rear_left_motor, motor_speed);
      wb_motor_set_velocity(front_left_motor, motor_speed);
    }

    // printf("left motor speed is %f\n", wb_motor_get_velocity(m1_motor));
    // printf("right motor speed is %f\n", wb_motor_get_velocity(m2_motor));
    // if (vel_y > 0.01){
    //   wb_motor_set_velocity(left_motor, MAX_SPEED+vel_y);
    //   wb_motor_set_velocity(right_motor, MAX_SPEED);
    // }

    // sleep(0.5);

  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}