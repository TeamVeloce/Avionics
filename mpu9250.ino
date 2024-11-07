#include <Wire.h>
#include <MPU9250.h>

// Kalman Filter class definition
class KalmanFilter {
  public:
    KalmanFilter(float q = 0.01, float r = 0.01, float p = 1, float k = 0) 
      : Q(q), R(r), P(p), K(k), x_hat(0), x_hat_prev(0) {}

    float update(float measurement) {
      // Prediction update
      x_hat = x_hat_prev;
      P = P + Q;

      // Measurement update
      K = P / (P + R);
      x_hat = x_hat + K * (measurement - x_hat);
      P = (1 - K) * P;

      x_hat_prev = x_hat;
      return x_hat;
    }

    void setProcessNoise(float q) { Q = q; }
    void setMeasurementNoise(float r) { R = r; }
    void setEstimateError(float p) { P = p; }
    
  private:
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float P; // Estimate error covariance
    float K; // Kalman gain
    float x_hat; // Filtered estimate
    float x_hat_prev; // Previous estimate
};

// Create instances of KalmanFilter for pitch and roll
KalmanFilter kalmanX(0.01, 0.01, 1, 0);
KalmanFilter kalmanY(0.01, 0.01, 1, 0);

// Create an instance of the MPU9250 class
MPU9250 mpu;

// Calibration offsets
float gyroXoffset = 0;
float gyroYoffset = 0;

// Timing variables
unsigned long lastTime = 0;
const float dt = 0.01; // Time step in seconds (adjust as necessary)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.setup(0x68);  // Set the MPU9250 I2C address

  // Check if MPU9250 is connected
  if (!mpu.isConnected()) {
    Serial.println("MPU9250 connection failed");
    while (1);
  }

  // Read initial gyro offsets
  for (int i = 0; i < 1000; i++) {
    mpu.update();
    gyroXoffset += mpu.getGyroX();
    gyroYoffset += mpu.getGyroY();
    delay(3);
  }
  gyroXoffset /= 1000.0;
  gyroYoffset /= 1000.0;

  lastTime = millis();
}

void loop() {
  // Read accelerometer and gyroscope data
  mpu.update();

  // Get raw gyroscope values
  float gyroX = (mpu.getGyroX() - gyroXoffset) / 65.536; // Adjust scale as needed
  float gyroY = (mpu.getGyroY() - gyroYoffset) / 65.536; // Adjust scale as needed

  // Get accelerometer values
  float ax_g = mpu.getAccX();
  float ay_g = mpu.getAccY();
  float az_g = mpu.getAccZ();

  // Calculate pitch and roll angles from accelerometer
  float pitch_acc = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;
  float roll_acc = atan2(ay_g, az_g) * 180 / PI;

  // Get the current time
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Time difference in seconds

  // Integrate gyro data to get angles
  static float angleX = 0;
  static float angleY = 0;

  angleX += gyroX * dt;
  angleY += gyroY * dt;

  // Apply Kalman filter
  float filteredAngleX = kalmanX.update(pitch_acc);
  float filteredAngleY = kalmanY.update(roll_acc);

  // Print angles
  Serial.print("Filtered Angle X: ");
  Serial.print(filteredAngleX);
  Serial.print(" degrees, Filtered Angle Y: ");
  Serial.print(filteredAngleY);
  Serial.println(" degrees");

  lastTime = currentTime;
  delay(10); // Delay to prevent excessive serial output
}