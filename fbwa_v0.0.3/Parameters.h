#ifndef PARAMETERS_H
#define PARAMETERS_H

// Connection
int param_baud = 115200;

// PID
double param_pid_roll_kp = 3;
double param_pid_roll_ki = 0.05;
double param_pid_roll_kd = 0.25;

double param_pid_pitch_kp = 3;
double param_pid_pitch_ki = 0.05;
double param_pid_pitch_kd = 0.25;

// Servo
int param_servo_1_max = 1900;
int param_servo_1_min = 1100;
int param_servo_1_trim = 1500;
int param_servo_1_rev = 1;

int param_servo_2_max = 1900;
int param_servo_2_min = 1100;
int param_servo_2_trim = 1500;
int param_servo_2_rev = 1;

int param_servo_4_max = 1900;
int param_servo_4_min = 1100;
int param_servo_4_trim = 1500;
int param_servo_4_rev = 1;

#endif
