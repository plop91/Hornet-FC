/** @file      HornetFC.cpp
 *  @brief    Flight Controller Object, has one of each enabled sub-system
 *  @details
 *  @author    Ian Sodersjerna
 *  @version
 *  @date      2022
 *  @pre
 *  @bug
 *  @warning
 *  @copyright GNU Public License.
 */
#include <Arduino.h>
#include "HornetFC.h"
#include "config.h"

HornetFC::HornetFC()
{
  /**
   * @brief Initialize and connect to Modules.
   *
   */
  // Create logging object
  this->hornet_logger = new HornetLOGGER();

  // set default state
  this->current_state = this->default_state;

  // Create imu object and connect to it.
  this->hornet_imu = new HornetIMU(this->hornet_logger);
  this->hornet_imu->begin(&gyroX, &gyroY, &gyroZ, &accX, &accY, &accZ, &magX, &magY, &magZ, &angleX, &angleY, &angleZ);

  // Create Sbus object and connect to it.
  this->hornet_sbus = new HornetSBUS(this->hornet_logger);

#if (defined SBUS_8 && !defined SBUS_16)
  this->hornet_sbus->begin(&channel_1_pwm, &channel_2_pwm, &channel_3_pwm, &channel_4_pwm, &channel_5_pwm, &channel_6_pwm, &channel_7_pwm, &channel_8_pwm);
#endif // SBUS_8

#ifdef SBUS_16
  this->hornet_sbus->begin(&channel_1_pwm, &channel_2_pwm, &channel_3_pwm, &channel_4_pwm, &channel_5_pwm, &channel_6_pwm, &channel_7_pwm, &channel_8_pwm,
                           &channel_9_pwm, &channel_10_pwm, &channel_11_pwm, &channel_12_pwm, &channel_13_pwm, &channel_14_pwm, &channel_15_pwm, &channel_16_pwm);
#endif // SBUS_8

#ifdef GPS_ENBL
  // Create GPS object and connect to it
  this->hornet_gps = new HornetGPS(this->hornet_logger);
  this->hornet_gps->begin();
#endif // GPS_ENBL

#ifdef BARO_ENBL
  // Create barometer object and connect to it
  this->hornet_baro = new HornetBarometer(this->hornet_logger);
  this->hornet_baro->begin();
#endif // BARO_ENBL

#ifdef BUZZER_ENBL
  // create a buzzer object
  hornet_buzzer = new HornetBuzzer(this->hornet_logger);
#endif // BUZZER_ENBL

// define LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

#ifdef ONBOARD_PWM_OUTPUTS
  this->hornet_logger->debug("setup PWM outputs!");
  // Attach servos to servo pins
  // Pin, min PWM value, max PWM value
  servo1.attach(servo1Pin, 1000, 2000);
  servo2.attach(servo2Pin, 1000, 2000);
  servo3.attach(servo3Pin, 1000, 2000);
  servo4.attach(servo4Pin, 1000, 2000);
  servo5.attach(servo5Pin, 1000, 2000);
  servo6.attach(servo6Pin, 1000, 2000);
  servo7.attach(servo7Pin, 1000, 2000);
  servo8.attach(servo8Pin, 1000, 2000);
  servo9.attach(servo9Pin, 1000, 2000);
  servo10.attach(servo10Pin, 1000, 2000);
  this->hornet_logger->debug("setup PWM outputs complete!");
#endif // ONBOARD_PWM_OUTPUTS

#ifdef ONBOARD_PWM_MOTORS
  this->hornet_logger->debug("setup PWM Motors!");
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
  pinMode(motor5Pin, OUTPUT);
  pinMode(motor6Pin, OUTPUT);
  pinMode(motor7Pin, OUTPUT);
  pinMode(motor8Pin, OUTPUT);
  pinMode(motor9Pin, OUTPUT);
  pinMode(motor10Pin, OUTPUT);
  this->hornet_logger->debug("setup PWM Motors complete!");
#endif // ONBOARD_PWM_MOTORS

#ifdef PCA9685
  this->hornet_logger->debug("setup PCA9685!");
  this->pwm_device = new Adafruit_PWMServoDriver();
  this->pwm_device->begin();
  this->pwm_device->setOscillatorFrequency(27000000);
  this->pwm_device->setPWMFreq(50);
  this->hornet_logger->debug("setup PCA9685 complete!");
#endif // PCA9685

#ifdef BUZZER_ENBL
  hornet_buzzer->buzzer_play_hp();
#endif
}

void HornetFC::step()
{
  /**
   * @brief Main loop of the flight controller,
   *
   */

  this->begin_time = millis();
  this->hornet_logger->begin_step();

  this->hornet_imu->update();
  this->hornet_sbus->update();

  switch (this->current_state)
  {
  case state_idle:
    this->idle();
    break;
  case state_rc_flight:
    this->rc_flight();
    break;
  case state_automatic:
    this->automatic();
    break;
  case state_emergency:
    this->emergency();
    break;
  }

  this->step_counter++;
  this->end_time = millis();

#ifdef DEBUG
  this->hornet_logger->end_step(this->end_time - this->begin_time);
#endif // DEBUG
}

#ifdef BLINK_ENBL
void HornetFC::blink()
{
  /**
   * @brief Blink built in led to indicate loop execution
   *
   */
  if (begin_time - blink_counter > blink_delay)
  {
    blink_counter = micros();
    digitalWrite(LEDR, blinkAlternate);

    if (blinkAlternate == 1)
    {
      blinkAlternate = 0;
      blink_delay = 100000;
    }
    else if (blinkAlternate == 0)
    {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}
#endif // BLINK_ENBL

void HornetFC::write_all_servos()
{
  /**
   * @brief Writes PWM values to servos
   *
   */

#ifdef ONBOARD_PWM_OUTPUTS
  servo1.write(this->s1_command_PWM);
  servo2.write(this->s2_command_PWM);
  servo3.write(this->s3_command_PWM);
  servo4.write(this->s4_command_PWM);
  servo5.write(this->s5_command_PWM);
  servo6.write(this->s6_command_PWM);
  this->hornet_logger->debug(this->s6_command_PWM);
  servo7.write(this->s7_command_PWM);
  servo8.write(this->s8_command_PWM);
  // servo9.write(this->s9_command_PWM);
  // servo10.write(this->s10_command_PWM);
#endif // ONBOARD_PWM_OUTPUTS

#ifdef ONBOARD_PWM_MOTORS
  analogWrite(this->servo1Pin, this->servo_pwm1_val);
  analogWrite(this->servo2Pin, this->servo_pwm2_val);
  analogWrite(this->servo3Pin, this->servo_pwm3_val);
  analogWrite(this->servo4Pin, this->servo_pwm4_val);
  analogWrite(this->servo5Pin, this->servo_pwm5_val);
  analogWrite(this->servo6Pin, this->servo_pwm6_val);
  analogWrite(this->servo7Pin, this->servo_pwm7_val);
  analogWrite(this->servo8Pin, this->servo_pwm8_val);
  analogWrite(this->servo9Pin, this->servo_pwm9_val);
  analogWrite(this->servo10Pin, this->servo_pwm10_val);
#endif // ONBOARD_PWM_MOTORS
}

void HornetFC::loopRate(int freq)
{
  /**
   * @brief Times loops and attempts to keep loops running in consistant time.
   *
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - begin_time))
  {
    checker = micros();
  }
}

float HornetFC::invSqrt(float x)
{
  return 1.0 / sqrtf(x);
}

void HornetFC::controlMixer()
{
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthru. mX_command_scaled and sX_command scaled variables are used in scaleCommands()
   * in preparation to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *thro_des - direct throttle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */

  // Armed
  if (channel_6_pwm < 1500)
  {
    this->hornet_logger->debug("Armed");
    // Hover mode
    if (channel_5_pwm > 1700)
    {
      this->hornet_logger->debug("Hover mode");
      s1_command_scaled = thro_des + pitch_PID;
      s2_command_scaled = thro_des - pitch_PID + roll_PID;
      s3_command_scaled = thro_des - pitch_PID - roll_PID;
      s4_command_scaled = 0.5 + yaw_PID;
      s5_command_scaled = 0;
      s6_command_scaled = 1;
      s7_command_scaled = 0.5;
      s8_command_scaled = 0.5;
    }
    // Transition
    else if ((channel_5_pwm < 1700) & (channel_5_pwm > 1300))
    {
      this->hornet_logger->debug("Transition mode");
      s1_command_scaled = thro_des + pitch_PID;
      s2_command_scaled = thro_des - pitch_PID + roll_PID;
      s3_command_scaled = thro_des - pitch_PID - roll_PID;
      s4_command_scaled = 0.5 + yaw_PID;
      s5_command_scaled = 0.5 - pitch_PID + roll_PID;
      s6_command_scaled = 0.5 + pitch_PID + roll_PID;
      s7_command_scaled = 0.3;
      s8_command_scaled = 0.7;
    }
    // Forward flight
    else
    {
      this->hornet_logger->debug("Flight mode");
      s1_command_scaled = thro_des + pitch_PID;
      s2_command_scaled = thro_des - pitch_PID + roll_PID;
      s3_command_scaled = thro_des - pitch_PID - roll_PID;
      s4_command_scaled = 0.5;
      s5_command_scaled = 0.5 - pitch_PID + roll_PID;
      s6_command_scaled = 0.5 + pitch_PID + roll_PID;
      s7_command_scaled = 0.5 - pitch_PID + roll_PID;
      s8_command_scaled = 0.5 + pitch_PID + roll_PID;
    }
  }
  else
  {
    this->hornet_logger->debug("Disarmed");
    s1_command_scaled = 0;
    s2_command_scaled = 0;
    s3_command_scaled = 0;
    s4_command_scaled = 0.5;
    s5_command_scaled = 0;
    s6_command_scaled = 1;
    s7_command_scaled = 0.5;
    s8_command_scaled = 0.5;
  }
}

void HornetFC::getDesState()
{
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0) / 1000.0; // Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0) / 500.0;  // Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0) / 500.0; // Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0) / 500.0;   // Between -1 and 1
  roll_passthru = roll_des / 2.0;               // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0;             // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0;                 // Between -0.5 and 0.5

  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);               // Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       // Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void HornetFC::controlANGLE()
{
  // DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stabilize on angle set-point based on desired states roll_des, pitch_des, and yaw_des computed in
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implemented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stabilized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */

  // Roll
  error_roll = roll_des - angleX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = gyroX;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - angleY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = gyroY;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stabilize on rate from GyroZ
  error_yaw = yaw_des - gyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void HornetFC::controlRATE()
{
  // DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  // Roll
  error_roll = roll_des - gyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll + Kd_roll_rate * derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - gyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch + Kd_pitch_rate * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stabilize on rate from GyroZ
  error_yaw = yaw_des - gyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm < 1060)
  { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  gyroX_prev = gyroX;
  // Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  gyroY_prev = gyroY;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void HornetFC::scaleCommands()
{
  // DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated
   * which are used to command the servos.
   */
#ifdef ONBOARD_PWM_MOTORS
  // Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled * 125 + 125;
  m2_command_PWM = m2_command_scaled * 125 + 125;
  m3_command_PWM = m3_command_scaled * 125 + 125;
  m4_command_PWM = m4_command_scaled * 125 + 125;
  m5_command_PWM = m5_command_scaled * 125 + 125;
  m6_command_PWM = m6_command_scaled * 125 + 125;
  // Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);
#endif // ONBOARD_PWM_MOTORS

#ifdef ONBOARD_PWM_OUTPUTS
  // Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled * 180;
  s2_command_PWM = s2_command_scaled * 180;
  s3_command_PWM = s3_command_scaled * 180;
  s4_command_PWM = s4_command_scaled * 180;
  s5_command_PWM = s5_command_scaled * 180;
  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  s8_command_PWM = s8_command_scaled * 180;
  // Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 10, 170);
  s2_command_PWM = constrain(s2_command_PWM, 10, 170);
  s3_command_PWM = constrain(s3_command_PWM, 10, 170);
  s4_command_PWM = constrain(s4_command_PWM, 10, 170);
  s5_command_PWM = constrain(s5_command_PWM, 10, 170);
  s6_command_PWM = constrain(s6_command_PWM, 10, 170);
  s7_command_PWM = constrain(s7_command_PWM, 10, 170);
#endif // ONBOARD_PWM_OUTPUTS
}

void HornetFC::arm()
{
  // DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++)
  {
    write_all_servos();
    delay(2);
  }
}

void HornetFC::failSafe()
{
  // DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  // Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal)
    check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal)
    check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal)
    check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal)
    check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal)
    check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal)
    check6 = 1;

  // If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0)
  {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void HornetFC::calibrateESCs()
{
  // DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
  while (true)
  {
    prev_time = begin_time;
    begin_time = micros();
    dt = (begin_time - prev_time) / 1000000.0;

    digitalWrite(LEDR, HIGH); // LED on to indicate we are not in main loop

    // getCommands(); //Pulls current available radio commands
    this->hornet_sbus->update();
    failSafe();    // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

    getDesState(); // Convert raw commands to normalized values based on saturated control limits

    // getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    this->hornet_imu->update();

    // Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
    getDesState(); // Convert raw commands to normalized values based on saturated control limits

    m1_command_scaled = thro_des;
    m2_command_scaled = thro_des;
    m3_command_scaled = thro_des;
    m4_command_scaled = thro_des;
    m5_command_scaled = thro_des;
    m6_command_scaled = thro_des;
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
    s4_command_scaled = thro_des;
    s5_command_scaled = thro_des;
    s6_command_scaled = thro_des;
    s7_command_scaled = thro_des;
    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    // throttleCut(); //Directly sets motor commands to low based on state of ch5
#ifdef ONBOARD_PWM_OUTPUTS
    servo1.write(s1_command_PWM);
    servo2.write(s2_command_PWM);
    servo3.write(s3_command_PWM);
    servo4.write(s4_command_PWM);
    servo5.write(s5_command_PWM);
    servo6.write(s6_command_PWM);
    servo7.write(s7_command_PWM);
    write_all_servos(); // Sends command pulses to each motor pin using OneShot125 protocol
#endif                  // ONBOARD_PWM_OUTPUTS

    // printRadioData(); //Radio pwm values (expected: 1000 to 2000)

    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
  }
}

void HornetFC::idle()
{
  /**
   * @brief Ground idle state, used to calibrate and setup vehicle for flight
   *
   */

  this->hornet_imu->print();
  this->hornet_sbus->print();

#ifdef GPS_ENBL
  this->hornet_gps->update();
  this->hornet_gps->print();
#endif // GPS_ENBL

#ifdef BARO_ENBL
  this->hornet_baro->update();
  this->hornet_baro->print();
#endif // BARO_ENBL

#ifdef LOOP_DELAY
  delay(LOOP_DELAY);
#endif // LOOP_DELAY

#ifdef BLINK_ENBL
  this->blink();
#endif
}

void HornetFC::rc_flight()
{
  /**
   * @brief RC controlled flight state
   */

  // Desired state
  this->getDesState();

  // PID
  this->controlANGLE();

  // CONTROL MIX
  this->controlMixer();

  // Scale Commands
  this->scaleCommands();

  // Actuate Outputs
  this->write_all_servos();

#ifdef BLINK_ENBL
  this->blink();
#endif
}

void HornetFC::automatic()
{
  /**
   * @brief Automatic flight state
   *
   * @todo add waypoint mission style AI
   */
#ifdef GPS_ENBL
  if (this->begin_time - this->last_gps_time >= GPS_FREQ)
  {
    this->hornet_gps->update();
    this->last_gps_time = millis();
  }
#endif // GPS_ENBL
}

void HornetFC::emergency()
{
  /**
   * @brief Emergency
   *
   * @todo add emergency state
   */
}
