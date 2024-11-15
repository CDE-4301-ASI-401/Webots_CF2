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
#include <webots/led.h>
#include <webots/pen.h>
#include <webots/compass.h>
#include "SGBA.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


//#define DEBUG

#define MAX_SPEED 5
#define NUMBER_OF_INFRARED_SENSORS 4
#define WALL_DISTANCE 0.1
#define DESIRED_HEADING_ANGLE -0.7
#define HEADING_INCREMENT 4

// SGBA variables  
float vel_x, vel_y, vel_w;
float rssi_angle;
int state_wf;
float range_front_value, range_back_value, range_left_value, range_right_value;

static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
  // turret sensors
  "front infrared sensor", "left infrared sensor", "right infrared sensor", "rear infrared sensor",
  };


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


  // get and enable the infrared sensors
  WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
  for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
    infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
    wb_distance_sensor_enable(infrared_sensors[i], time_step);
  }

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

  WbDeviceTag compass = wb_robot_get_device("Compass");
  wb_compass_enable(compass, time_step);


  // get the led actuators
  WbDeviceTag leds[3] = {wb_robot_get_device("front left led"), wb_robot_get_device("front right led"),
                         wb_robot_get_device("rear led")};

  // get the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor, right_motor;
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  //WbDeviceTag = wb_robot_get_device("pen");

  // store the last time a message was displayed
  int last_display_second = 0;

  double get_bearing_in_degrees() {
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[1], north[0]);
    double bearing = rad;
    // double bearing = (rad - 1.5708) / M_PI * 180.0;
    // if (bearing < 0.0)
    //   bearing = bearing + 360.0;
    return -bearing;
  }
  // MAIN loop
  while (wb_robot_step(time_step) != -1) {
    // display some sensor data every second
    // and change randomly the led colors
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second) {
      last_display_second = display_second;

      #ifdef DEBUG
      printf("time = %d [s]\n", display_second);
      #endif
      //for (i = 0; i < 5; ++i)
      //  printf("- ultrasonic sensor('%s') = %f [m]\n", ultrasonic_sensors_names[i],
      //         wb_distance_sensor_get_value(ultrasonic_sensors[i]));
      for (i = 0; i < NUMBER_OF_INFRARED_SENSORS; i++)
        //#ifdef DEBUG
        printf("- infrared sensor('%s') = %f [m]\n", infrared_sensors_names[i],
               wb_distance_sensor_get_value(infrared_sensors[i]));
          //     #endif

      for (i = 0; i < 3; ++i)
        wb_led_set(leds[i], 0xFFFFFF & rand());
    }

    //get gps
    double x_global = wb_gps_get_values(gps)[0];
    double y_global = wb_gps_get_values(gps)[1];

    //get ranges
    range_front_value = wb_distance_sensor_get_value(infrared_sensors[0]);
    range_left_value = wb_distance_sensor_get_value(infrared_sensors[1]);
    range_right_value = wb_distance_sensor_get_value(infrared_sensors[2]);
    range_back_value = wb_distance_sensor_get_value(infrared_sensors[3]);
    float heading = (float)get_bearing_in_degrees();
    #ifdef DEBUG
    printf("bearing in rad: %f\n", heading);
    #endif
    //float heading = 0.0;
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
      wb_motor_set_velocity(right_motor, vel_w);
      wb_motor_set_velocity(left_motor, -vel_w);  
    }
    ///turn to find wall (does not have vel_w)
    else if ((state_wf==4 && state==3)){
      wb_motor_set_velocity(right_motor, direction * MAX_SPEED);
      wb_motor_set_velocity(left_motor, -direction * MAX_SPEED);  
    }   
    // else if (vel_y>0.1||vel_y<-0.1){
    //   wb_motor_set_velocity(left_motor,  MAX_SPEED-0.05*vel_y);
    //   wb_motor_set_velocity(right_motor, MAX_SPEED+0.05*vel_y);
    // } 
    else{
      wb_motor_set_velocity(left_motor,  MAX_SPEED - 2 * vel_w * MAX_SPEED - 0.05 * vel_y);
      wb_motor_set_velocity(right_motor, MAX_SPEED + 2 * vel_w * MAX_SPEED + 0.05 * vel_y);
      // wb_motor_set_velocity(left_motor,  MAX_SPEED);
      // wb_motor_set_velocity(right_motor, MAX_SPEED);
    }
    printf("left motor speed is %f\n", wb_motor_get_velocity(left_motor));
    printf("right motor speed is %f\n", wb_motor_get_velocity(right_motor));
    // if (vel_y > 0.01){
    //   wb_motor_set_velocity(left_motor, MAX_SPEED+vel_y);
    //   wb_motor_set_velocity(right_motor, MAX_SPEED);
    // }

    //sleep(0.5);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
