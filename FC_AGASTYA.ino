//================================================================================================================//
//                                         INCLUDE LIBRARIES                                                      //
//================================================================================================================//
#include <Wire.h>             // I2C communication library for talking to sensors like the MPU-6050
#include <PulsePosition.h>    // Library to read PPM signals from an RC receiver
#include <Servo.h>            // Library to control ESCs (Electronic Speed Controllers) like servos

//================================================================================================================//
//                                        GYROSCOPE VARIABLES                                                     //
//================================================================================================================//
float RateRoll, RatePitch, RateYaw;                               // Current angular rates from the gyro (degrees/second)
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw; // Calibration offsets for the gyro
int RateCalibrationNumber;                                        // Counter for the calibration loop

//================================================================================================================//
//                                        RECEIVER & MOTOR VARIABLES                                              //
//================================================================================================================//
Servo esc1, esc2, esc3, esc4;                                     // Servo objects for the four ESCs/motors
PulsePositionInput ReceiverInput(RISING);                         // Object to handle PPM input from the RC receiver
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};                 // Array to store the pulse widths for each receiver channel
int ChannelNumber = 0;                                            // Number of channels detected from the receiver

//================================================================================================================//
//                                        FLIGHT & SYSTEM VARIABLES                                               //
//================================================================================================================//
float Voltage, Current;                   // Battery voltage and current draw
uint32_t LoopTimer;                       // Timer to ensure the main loop runs at a consistent rate (e.g., 250Hz)
float InputRoll, InputThrottle, InputPitch, InputYaw; // Processed inputs from the PID controllers
float MotorInput1, MotorInput2, MotorInput3, MotorInput4; // Final pulse width values sent to the ESCs

//================================================================================================================//
//                                        PID CONTROLLER VARIABLES (RATE MODE)                                    //
//================================================================================================================//
// Desired rates are the target angular velocities, typically set by the pilot's sticks (in acro/rate mode)
// or by the angle PID controller (in self-level mode).
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

// Error is the difference between the desired rate and the actual rate from the gyro.
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

// Previous errors and I-term accumulations are stored for the next PID calculation.
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// The PID controller's output is stored in this array.
float PIDReturn[] = {0, 0, 0};

// --- RATE PID GAINS (Tuning Parameters) ---
// These values determine how the drone reacts to control inputs and disturbances.
// P (Proportional): Reacts to the current error. Higher P = sharper response.
float PRateRoll = 0.6;
float PRatePitch = PRateRoll; // Pitch P-gain is often the same as Roll
float PRateYaw = 0;           // Yaw is often less responsive and needs different tuning.

// I (Integral): Corrects for steady-state error over time. Helps resist drift.
float IRateRoll = 0.0;
float IRatePitch = IRateRoll;
float IRateYaw = 0;

// D (Derivative): Dampens the response by reacting to the rate of change of the error. Reduces overshoot.
float DRateRoll = 0.000;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

//================================================================================================================//
//                                     ACCELEROMETER & KALMAN FILTER VARIABLES                                    //
//================================================================================================================//
// Raw accelerometer data and calculated angles
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Kalman filter variables for Roll axis
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;

// Kalman filter variables for Pitch axis
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 2 * 2;

// Output array for the Kalman filter function
float Kalman1DOutput[] = {0, 0};

//================================================================================================================//
//                                        PID CONTROLLER VARIABLES (ANGLE/LEVEL MODE)                             //
//================================================================================================================//
// Desired angles are the target orientation of the drone, set by the pilot's sticks.
float DesiredAngleRoll, DesiredAnglePitch;

// Error is the difference between the desired angle and the actual (Kalman filtered) angle.
float ErrorAngleRoll, ErrorAnglePitch;

// Previous errors and I-term accumulations for the angle PID controller.
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

// --- ANGLE PID GAINS (Tuning Parameters for Self-Leveling) ---
float PAngleRoll = 2.8;
float PAnglePitch = 3.0;
float IAngleRoll = 0.1;
float IAnglePitch = 0.1;
float DAngleRoll = 0.001;
float DAnglePitch = 0.001;


//================================================================================================================//
//                                        KALMAN FILTER FUNCTION                                                  //
//================================complements the accelerometer and gyroscope data to get a more stable angle estimate.
// It fuses the noisy but drift-free accelerometer angle with the clean but drifty gyroscope rate.
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Prediction stage: estimate the current state based on the previous state and the gyro input.
  KalmanState = KalmanState + 0.004 * KalmanInput; // 0.004 is the loop time (4ms)
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  // Update stage: correct the predicted state with the new measurement from the accelerometer.
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Store the results.
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

//================================================================================================================//
//                                        BATTERY MONITORING FUNCTION                                             //
//================================================================================================================//
// Reads analog pins to measure battery voltage and current.
void battery_voltage(void) {
  Voltage = (float)analogRead(15) / 62; // Read pin A15 and convert to voltage
  Current = (float)analogRead(21) * 0.089; // Read pin A21 and convert to current
}

//================================================================================================================//
//                                        RECEIVER READING FUNCTION                                               //
//================================================================================================================//
// Checks for and reads new data from the RC receiver.
void read_receiver(void) {
  ChannelNumber = ReceiverInput.available(); // Check if new channel data is available
  if (ChannelNumber > 0) {
    // Loop through all available channels and store their pulse width values.
    for (int i = 1; i <= ChannelNumber; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
}

//================================================================================================================//
//                                        IMU (GYRO/ACCEL) READING FUNCTION                                       //
//================================================================================================================//
// Communicates with the MPU-6050 Inertial Measurement Unit (IMU) to get sensor data.
void gyro_signals(void) {
  // --- Configure and Read Accelerometer ---
  Wire.beginTransmission(0x68); // Start I2C communication with the MPU-6050 (address 0x68)
  Wire.write(0x1A);             // Access DLPF (Digital Low Pass Filter) register
  Wire.write(0x05);             // Set the DLPF to a low setting
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);             // Access accelerometer configuration register
  Wire.write(0x10);             // Set accelerometer sensitivity to +/- 8g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);             // Set the starting register for accelerometer data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);    // Request 6 bytes of data (X, Y, Z, high & low bytes)
  int16_t AccXLSB = Wire.read() << 8 | Wire.read(); // Combine bytes for X-axis
  int16_t AccYLSB = Wire.read() << 8 | Wire.read(); // Combine bytes for Y-axis
  int16_t AccZLSB = Wire.read() << 8 | Wire.read(); // Combine bytes for Z-axis

  // --- Configure and Read Gyroscope ---
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);             // Access gyroscope configuration register
  Wire.write(0x8);              // Set gyroscope sensitivity to +/- 500 degrees/sec
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);             // Set the starting register for gyro data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);    // Request 6 bytes of data
  int16_t GyroX = Wire.read() << 8 | Wire.read(); // Combine bytes for X-axis (Roll)
  int16_t GyroY = Wire.read() << 8 | Wire.read(); // Combine bytes for Y-axis (Pitch)
  int16_t GyroZ = Wire.read() << 8 | Wire.read(); // Combine bytes for Z-axis (Yaw)

  // --- Convert Raw Sensor Data to Usable Units ---
  // Convert raw gyro data to degrees/second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Convert raw accelerometer data to g's and apply calibration offsets
  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.06;
  AccZ = (float)AccZLSB / 4096 + 0.13;

  // Calculate roll and pitch angles from accelerometer data (in degrees)
  // This is noisy and only accurate when the drone is not accelerating.
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}


//================================================================================================================//
//                                        PID CALCULATION FUNCTION                                                //
//================================================================================================================//
// This function calculates the output of a PID controller for a given axis.
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  // Proportional term
  float Pterm = P * Error;

  // Integral term with anti-windup (trapezoidal integration)
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;       // Clamp the I-term to prevent integral windup
  else if (Iterm < -400) Iterm = -400;

  // Derivative term
  float Dterm = D * (Error - PrevError) / 0.004;

  // Sum the terms to get the final PID output
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400; // Clamp the total output
  else if (PIDOutput < -400) PIDOutput = -400;

  // Store the results for use in the main loop and for the next iteration
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error; // The current error becomes the previous error for the next loop
  PIDReturn[2] = Iterm; // The current I-term becomes the previous I-term
}

//================================================================================================================//
//                                        PID RESET FUNCTION                                                      //
//================================================================================================================//
// Resets the PID controller's state variables. This is crucial when disarming
// to prevent the I-term from accumulating and causing a lurch on re-arming.
void reset_pid(void) {
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}

//================================================================================================================//
//                                        SETUP FUNCTION                                                          //
//================================================================================================================//
// This code runs once at startup.
void setup() {
  // --- Initialize Hardware ---
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH); // Set up an indicator LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // Onboard LED

  Wire.setClock(400000); // Set I2C clock speed to 400kHz for faster sensor communication
  Wire.begin();
  delay(250); // Wait for sensors to stabilize

  // Wake up the MPU-6050, as it starts in sleep mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Access power management register
  Wire.write(0x00); // Set to 0 to wake up the sensor
  Wire.endTransmission();

  // --- Gyroscope Calibration ---
  // The gyro has a natural bias (drift) that needs to be measured and subtracted.
  // This loop takes 2000 readings and averages them to find the bias.
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1); // Wait a moment between readings
  }
  // Calculate the average bias
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // --- ESC and Receiver Setup ---
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  ReceiverInput.begin(15); // Initialize the PPM receiver on pin 15

  // Wait for the arming switch on the transmitter to be in the 'armed' position.
  // This is a safety feature to prevent the motors from spinning up accidentally.
  // Here, it seems channel 8 is used for arming.
  while (ReceiverValue[7] < 1600) {
    read_receiver();
    delay(4);
  }

  // Attach the ESCs to their respective pins.
  esc1.attach(3);
  esc2.attach(4);
  esc3.attach(5);
  esc4.attach(6);

  // Start the main loop timer.
  LoopTimer = micros();
}

//================================================================================================================//
//                                        MAIN LOOP                                                               //
//================================================================================================================//
// This code runs repeatedly at a high speed (target is 250Hz, or every 4000 microseconds).
void loop() {
  // --- SENSOR FUSION ---
  gyro_signals(); // Get the latest data from the MPU-6050

  // Apply the calibration offsets to the raw gyro rates.
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Run the Kalman filter to get a stable angle estimate for roll and pitch.
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // --- READ CONTROLLER INPUTS ---
  read_receiver(); // Get the latest commands from the pilot.

  // Convert receiver pulse widths (typically 1000-2000) into desired angles/rates.
  // These lines are for self-leveling mode.
  DesiredAngleRoll = 0.09 * (ReceiverValue[0] - 1500);  // Channel 1 for Roll
  DesiredAnglePitch = 0.09 * (ReceiverValue[1] - 1500); // Channel 2 for Pitch
  InputThrottle = ReceiverValue[2];                      // Channel 3 for Throttle
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);   // Channel 4 for Yaw Rate

  // --- PID CONTROLLER CALCULATIONS ---
  // ** This code is configured for RATE (ACRO) MODE. **
  // The angle PID controller is commented out. In this mode, the pilot directly controls
  // the drone's rate of rotation, not its angle.

  // // ANGLE MODE PID (Currently commented out)
  // // First, calculate the error between the desired angle and the actual angle.
  // ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  // ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  // // The output of the angle PID controller becomes the *input* (the desired rate) for the rate PID controller.
  // pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  // DesiredRateRoll = PIDReturn[0];
  // PrevErrorAngleRoll = PIDReturn[1];
  // PrevItermAngleRoll = PIDReturn[2];
  // pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  // DesiredRatePitch = PIDReturn[0];
  // PrevErrorAnglePitch = PIDReturn[1];
  // PrevItermAnglePitch = PIDReturn[2];


  // RATE MODE PID
  // Calculate the error between the desired rate and the actual rate from the gyro.
  // In pure rate mode, DesiredRateRoll and DesiredRatePitch would be set directly from the receiver,
  // but here they are implicitly 0 because the angle controller is off.
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Calculate the PID output for each axis.
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // --- MOTOR MIXING ---
  // Mix the throttle and PID outputs to determine the speed for each of the four motors.
  // This is for a standard "X" quadcopter configuration.
  if (InputThrottle > 1800) InputThrottle = 1800; // Limit max throttle

  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // Front-Right
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // Rear-Right
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // Front-Left
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // Rear-Left

  // Limit the motor outputs to the valid range for the ESCs (e.g., 1000-2000 microseconds).
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  int ThrottleIdle = 1180; // Minimum throttle to keep motors spinning
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // --- THROTTLE CUT-OFF / DISARM ---
  // If the throttle stick is all the way down, cut power to the motors and reset the PIDs.
  int ThrottleCutOff = 900;
  if (ReceiverValue[2] < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid(); // Reset PID integrals to prevent issues on re-arming
  }

  // --- WRITE TO MOTORS ---
  // Send the final calculated pulse width signals to the ESCs.
  esc1.writeMicroseconds(MotorInput1);
  esc2.writeMicroseconds(MotorInput2);
  esc3.writeMicroseconds(MotorInput3);
  esc4.writeMicroseconds(MotorInput4);

  // --- DEBUGGING OUTPUT (Optional) ---
  // Print motor values and rates to the serial monitor for tuning and debugging.
  // Serial.print("1: ");
  // Serial.print(MotorInput1);
  // Serial.print("2: ");
  // Serial.print(MotorInput2);
  // Serial.print("3: ");
  // Serial.print(MotorInput3);
  // Serial.print("4: ");
  // Serial.println(MotorInput4);
  // Serial.print("x: ");
  // Serial.print(RateRoll);
  // Serial.print("y: ");
  // Serial.print(RatePitch);
  // Serial.print("z: ");
  // Serial.println(RateYaw);

  // --- LOOP TIMING ---
  // Wait until 4000 microseconds (4ms) have passed since the start of the loop.
  // This ensures the loop runs at a consistent 250Hz, which is important for stable PID control.
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros(); // Reset the timer for the next loop iteration.
}
