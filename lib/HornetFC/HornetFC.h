/**
 * @file      HornetFC.h
 * @brief
 * @details
 * @author    Ian Sodersjerna
 * @version   0.0.1
 * @date      2022
 * @pre
 * @bug
 * @warning
 * @copyright GNU Public License.
 */
#ifndef HORNET_FC_H
#define HORNET_FC_H
#include <Arduino.h>
#include "config.h"

// General
#include <Wire.h>
#include <SPI.h>

// Required Modules
#include <HornetLOGGER.h>
#include <HornetSBUS.h>
#include <HornetIMU.h>

#if (defined ONBOARD_PWM_OUTPUTS || defined ONBOARD_PWM_MOTORS)
#ifndef ESPCAM
#include <Servo.h>
#else
#include <ESP32Servo.h>
#endif
#endif // ONBOARD_PWM_OUTPUTS

#ifdef PCA9685 // PCA9685
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600    // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400   // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50
#endif // PCA9685

#ifdef BARO_ENBL
#include <HornetBarometer.h>
#endif // BARO_ENBL

#ifdef GPS_ENBL
#include <HornetGPS.h>
#endif // GPS_ENBL

#ifdef BUZZER_ENBL
#include <HornetBuzzer.h>
#endif // BUZZER_ENBL

class HornetFC
{
  /**
   * @class HornetFC
   * @brief Represents a Hornet Flight Controller and its systems.
   *
   *
   */
public:
  HornetFC();  //! Constructor
  void step(); //! Main loop of program

private:
  enum states
  {
    state_idle,
    state_rc_flight,
    state_automatic,
    state_emergency
  };
  states default_state = state_idle;
  states current_state;

  // step counter
  int step_counter = 0;

// LEDs
#define LEDR 20
#define LEDG 41
#define LEDB 40

#ifdef ONBOARD_PWM_OUTPUTS
  // PWM variables (usually for control surfaces, use motor variables to control motors)
  const int servo1Pin = 0;
  const int servo2Pin = 1;
  const int servo3Pin = 2;
  const int servo4Pin = 3;
  const int servo5Pin = 4;
  const int servo6Pin = 5;
  const int servo7Pin = 6;
  const int servo8Pin = 7;
  const int servo9Pin = 8;
  const int servo10Pin = 9;

  int servo_pwm1_val = 0;
  int servo_pwm2_val = 0;
  int servo_pwm3_val = 0;
  int servo_pwm4_val = 0;
  int servo_pwm5_val = 0;
  int servo_pwm6_val = 0;
  int servo_pwm7_val = 0;
  int servo_pwm8_val = 0;
  int servo_pwm9_val = 0;
  int servo_pwm10_val = 0;

  Servo servo1;  //! Control for servo 1
  Servo servo2;  //! Control for servo 2
  Servo servo3;  //! Control for servo 3
  Servo servo4;  //! Control for servo 4
  Servo servo5;  //! Control for servo 5
  Servo servo6;  //! Control for servo 6
  Servo servo7;  //! Control for servo 7
  Servo servo8;  //! Control for servo 8
  Servo servo9;  //! Control for servo 9
  Servo servo10; //! Control for servo 10
#endif           // ONBOARD_PWM_OUTPUTS

#ifdef ONBOARD_PWM_MOTORS
  // PWM variables (usually for control surfaces, use motor variables to control motors)

  const int motor1Pin = 0;
  const int motor2Pin = 1;
  const int motor3Pin = 2;
  const int motor4Pin = 3;
  const int motor5Pin = 4;
  const int motor6Pin = 5;
  const int motor7Pin = 6;
  const int motor8Pin = 7;
  const int motor9Pin = 8;
  const int motor10Pin = 9;

  int motor_pwm1_val = 0; //!
  int motor_pwm2_val = 0;
  int motor_pwm3_val = 0;
  int motor_pwm4_val = 0;
  int motor_pwm5_val = 0;
  int motor_pwm6_val = 0;
  int motor_pwm7_val = 0;
  int motor_pwm8_val = 0;
  int motor_pwm9_val = 0;
  int motor_pwm10_val = 0;
#endif // ONBOARD_PWM_MOTORS

#ifdef PCA9685
  Adafruit_PWMServoDriver *pwm_device;
#endif // PCA9685

  // Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
  float B_madgwick = 0.04; // Madgwick filter parameter
  float B_accel = 0.14;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
  float B_gyro = 0.1;      // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
  float B_mag = 1.0;       // Magnetometer LP filter parameter

  // Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
  float MagErrorX = 0.0;
  float MagErrorY = 0.0;
  float MagErrorZ = 0.0;
  float MagScaleX = 1.0;
  float MagScaleY = 1.0;
  float MagScaleZ = 1.0;

  // IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
  float AccErrorX = 0.0;
  float AccErrorY = 0.0;
  float AccErrorZ = 0.0;
  float GyroErrorX = 0.0;
  float GyroErrorY = 0.0;
  float GyroErrorZ = 0.0;

  // Controller parameters (take note of defaults before modifying!):
  float i_limit = 25.0;  // Integrator saturation level, mostly for safety (default 25.0)
  float maxRoll = 30.0;  // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
  float maxPitch = 30.0; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
  float maxYaw = 160.0;  // Max yaw rate in deg/sec

  float Kp_roll_angle = 0.2;   // Roll P-gain - angle mode
  float Ki_roll_angle = 0.3;   // Roll I-gain - angle mode
  float Kd_roll_angle = 0.05;  // Roll D-gain - angle mode (has no effect on controlANGLE2)
  float B_loop_roll = 0.9;     // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
  float Kp_pitch_angle = 0.2;  // Pitch P-gain - angle mode
  float Ki_pitch_angle = 0.3;  // Pitch I-gain - angle mode
  float Kd_pitch_angle = 0.05; // Pitch D-gain - angle mode (has no effect on controlANGLE2)
  float B_loop_pitch = 0.9;    // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

  float Kp_roll_rate = 0.15;    // Roll P-gain - rate mode
  float Ki_roll_rate = 0.2;     // Roll I-gain - rate mode
  float Kd_roll_rate = 0.0002;  // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  float Kp_pitch_rate = 0.15;   // Pitch P-gain - rate mode
  float Ki_pitch_rate = 0.2;    // Pitch I-gain - rate mode
  float Kd_pitch_rate = 0.0002; // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

  float Kp_yaw = 0.3;     // Yaw P-gain
  float Ki_yaw = 0.05;    // Yaw I-gain
  float Kd_yaw = 0.00015; // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)


  // Normalized desired state:
  float thro_des, roll_des, pitch_des, yaw_des;
  float roll_passthru, pitch_passthru, yaw_passthru;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //! ## Time
  float dt;
  unsigned long begin_time, end_time, prev_time;

  //! ## Gyroscope variables
  float gyroX, gyroY, gyroZ;
  float gyroX_prev, gyroY_prev, gyroZ_prev;

  //! ## Accelerometer variables
  float accX, accY, accZ;
  float accX_prev, accY_prev, accZ_prev;

  //! ## Accelerometer variables
  float magX, magY, magZ;
  float magX_prev, magY_prev, magZ_prev;

  //! ## Angle variables
  float angleX, angleY, angleZ;
  float angleX_prev, angleY_prev, angleZ_prev;

  //! ## PID Variables
  float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
  float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
  float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

  //! ## Mixer variables

  //! ### Motor
  float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled, m7_command_scaled, m8_command_scaled;
  int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM, m7_command_PWM, m8_command_PWM;

  //! ### Servos
  float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled, s8_command_scaled;
  int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM, s8_command_PWM;

  //Receiver variables
#if (defined SBUS_8 || defined SBUS_16)
  unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
  unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev, channel_5_pwm_prev, channel_6_pwm_prev, channel_7_pwm_prev, channel_8_pwm_prev;
  unsigned long channel_1_fs = 1000; // thro
  unsigned long channel_2_fs = 1500; // ail
  unsigned long channel_3_fs = 1500; // elev
  unsigned long channel_4_fs = 1500; // rudd
  unsigned long channel_5_fs = 2000; // 
  unsigned long channel_6_fs = 2000; // 
  unsigned long channel_7_fs = 2000; // 
  unsigned long channel_8_fs = 2000; // 
#endif

#ifdef SBUS_16
  unsigned long channel_9_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm, channel_13_pwm, channel_14_pwm, channel_15_pwm, channel_16_pwm;
  unsigned long channel_9_pwm_prev, channel_10_pwm_prev, channel_11_pwm_prev, channel_12_pwm_prev, channel_13_pwm_prev, channel_14_pwm_prev, channel_15_pwm_prev, channel_16_pwm_prev;
  unsigned long channel_9_fs = 1000; // 
  unsigned long channel_10_fs = 1000; // 
  unsigned long channel_11_fs = 1000; // 
  unsigned long channel_12_fs = 1000; // 
  unsigned long channel_13_fs = 1000; // 
  unsigned long channel_14_fs = 1000; // 
  unsigned long channel_15_fs = 1000; // 
  unsigned long channel_16_fs = 1000; // 
#endif // SBUS_16

 

  // Logging
  HornetLOGGER *hornet_logger;

  // IMU
  HornetIMU *hornet_imu;

  // SBUS receiver
  HornetSBUS *hornet_sbus;

#ifdef BARO_ENBL
  // Barometer
  HornetBarometer *hornet_baro;
  unsigned long last_baro_time = 0;
  const unsigned long BARO_FREQ = 10;
#endif // BARO_ENBL

#ifdef GPS_ENBL // If GPS enabled
  // GPS
  HornetGPS *hornet_gps;
  unsigned long last_gps_time = 0;
  const unsigned long GPS_FREQ = 50;
#endif // GPS_ENBL

#ifdef BUZZER_ENBL
  // BUZZER
  HornetBuzzer *hornet_buzzer;
#endif // BUZZER_ENBL

  // States
  void idle();
  void rc_flight();
  void automatic();
  void emergency();

#ifdef BLINK_ENBL
  bool blinkAlternate;
  unsigned long blink_counter, blink_delay;
  void blink();
#endif

  void write_all_servos();

  void loopRate(int freq);

  float invSqrt(float x);

  // Control Functions

  void controlMixer();

  void getDesState();

  void controlANGLE();

  void controlRATE();

  void scaleCommands();

  void arm();

  void failSafe();

  // Calibration functions

  void calibrateESCs();

  void calibrateAttitude();
};

#endif // HORNET_FC_H
