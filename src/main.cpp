#include <LibRobus.h>


// left = 0, right = 1

// Speed calculation variables
float motor_0_rpm = 0.0;
float motor_1_rpm = 0.0;
uint32_t last_sample_time = 0;

// PID parameters for motor 0
float kp_0  = 0.007f;
float ki_0  = 0.0098f;
float kd_0  = 0.00001f;

static float prev_error_0 = 0.0f;
static float integral_0   = 0.0f;

// PID parameters for motor 1
float kp_1 = 0.007f;
float ki_1 = 0.0025f;
float kd_1 = 0.00001f;

static float prev_error_1 = 0.0f;
static float integral_1   = 0.0f;


// moving
unsigned long moveStartTime = 0;
bool movingForward = false;

// Function to calculate RPM from pulse count
float calculateRPM(int32_t pulse_count, float time_interval_seconds) {
  // RPM = (Pulses Counted * 60) / (dt * Pulses Per Revolution(3200))
  return (pulse_count * 60.0f) / (time_interval_seconds * 3200);
}

float pidControllerMotor0(float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  float P = kp_0 * error;

  integral_0 += error * dt;

  float I = ki_0 * integral_0;
  float D = kd_0 * ((error - prev_error_0) / dt);

  prev_error_0 = error;

  float output = P + I + D;

  return output;
}

float pidControllerMotor1(float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  float P = kp_1 * error;

  integral_1 += error * dt;

  float I = ki_1 * integral_1;
  float D = kd_1 * ((error - prev_error_1) / dt);

  prev_error_1 = error;
  float output = P + I + D;

  return output;
}

// minimize the oscillation at beginning
float rampTargetRPM(float targetRPM, uint32_t startTime, uint32_t currentTime) {
  uint32_t elapsed = currentTime - startTime;
  float rampTime = 1000; // 1 seconds ramp

  if (elapsed >= rampTime) return targetRPM;

  return targetRPM * (elapsed / rampTime);
}

void setup() {
  BoardInit();

  ENCODER_Reset(0);
  ENCODER_Reset(1);

  last_sample_time = millis();
}

void loop() {
  uint32_t current_time = millis();
  if (current_time - last_sample_time >= 10) {     // 10ms
    
    /*
    float dt = (current_time - last_sample_time) / 1000.0f;

    // Read and reset encoders
    int32_t pulses_0  = ENCODER_ReadReset(0);
    int32_t pulses_1 = ENCODER_ReadReset(1);

    // Calculate actual speeds
    motor_0_rpm  = calculateRPM(pulses_0,  dt);
    motor_1_rpm = calculateRPM(pulses_1, dt);

    // Compute PID outputs (set your RPM targets here)
    float target_rpm_0  = 20.0f;
    float target_rpm_1 = 20.0f;

    float power_0  = pidControllerMotor0(target_rpm_0,  motor_0_rpm,  dt);
    float power_1 = pidControllerMotor1(target_rpm_1, motor_0_rpm, dt);

    // Conditional motor enable based on bumper
    if (ROBUS_IsBumper(3)) {
      MOTOR_SetSpeed(0,  power_0);
      MOTOR_SetSpeed(1, power_1);
    } else {
      MOTOR_SetSpeed(0,  0.0f);
      MOTOR_SetSpeed(1, 0.0f);
    }
    */

    if (!movingForward) {
        if (ROBUS_IsBumper(3)) {
          movingForward = true;
          moveStartTime = current_time;
        }
      }

      if (movingForward) {
        float dt = (current_time - last_sample_time) / 1000.0f;

        // Update encoder values
        int32_t pulses_0  = ENCODER_ReadReset(0);
        int32_t pulses_1 = ENCODER_ReadReset(1);
        motor_0_rpm = calculateRPM(pulses_0, dt);
        motor_1_rpm = calculateRPM(pulses_1, dt);

        // Calculate PID outputs

        float target_rpm = rampTargetRPM(50.0f, moveStartTime, millis());

        float power_0  = pidControllerMotor0(target_rpm, motor_0_rpm, dt);
        float power_1 = pidControllerMotor1(target_rpm, motor_1_rpm, dt);

        MOTOR_SetSpeed(0, power_0);
        MOTOR_SetSpeed(1, power_1);

        if (current_time - moveStartTime >= 7000) {  // stop after 10 seconds
          MOTOR_SetSpeed(0, 0.0f);
          MOTOR_SetSpeed(1, 0.0f);
          movingForward = false;
        }
        last_sample_time = current_time;

      } else {
        MOTOR_SetSpeed(0, 0.0f);
        MOTOR_SetSpeed(1, 0.0f);
      }

      // Debug, but this slow down the speed of the calculation which cause the value we see here is not the actual value that we will see on real ground (fuck)
      Serial.print("time: "); Serial.print(current_time);
      Serial.print(", rpm0: "); Serial.print(motor_0_rpm,  5); 
      Serial.print(", rpm1: "); Serial.print(motor_1_rpm, 5);
      // Serial.print(", power0: "); Serial.print(power_0,  5); 
      // Serial.print(", power1: "); Serial.print(power_1, 5);
      Serial.print(", diff: "); Serial.print(motor_0_rpm-motor_1_rpm,  5);
      Serial.print(", error0: "); Serial.print(prev_error_0,  5); 
      Serial.print(", error1: "); Serial.println(prev_error_1, 5);

      last_sample_time = current_time;
  }
}
