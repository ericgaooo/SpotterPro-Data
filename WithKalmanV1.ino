#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <iostream>
#include <vector>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <stdexcept>
#define BNO055_SAMPLERATE_PERIOD_MS 10
#define VELOCITY_THRESHOLD 0.2    // Adjust this threshold for velocity calcs
#define CALIBRATION_RANGE 0.1     // Range for calibration
#define CALIBRATION_DURATION 600  // Duration in milliseconds for calibration

class KalmanFilter {

  public:

    /**
      Create a Kalman filter with the specified matrices.
        A - System dynamics matrix
        C - Output matrix
        Q - Process noise covariance
        R - Measurement noise covariance
        P - Estimate error covariance
    */
    KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
    );

    /**
      Create a blank estimator.
    */
    KalmanFilter();

    /**
      Initialize the filter with initial states as zero.
    */
    void init();

    /**
      Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0);

    /**
      Update the estimated state based on measured values. The
      time step is assumed to remain constant.
    */
    void update(const Eigen::VectorXd& y);

    /**
      Update the estimated state based on measured values,
      using the given time step and dynamics matrix.
    */
    void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

    /**
      Return the current state and time.
    */
    Eigen::VectorXd state() {
      return x_hat;
    };
    double time() {
      return t;
    };

  private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;
};

KalmanFilter::KalmanFilter(
  double dt,
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& C,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R,
  const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if (!initialized)
    Serial.println("Kalman filter not initialized");

  x_hat_new = A * x_hat;
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  x_hat_new += K * (y - C * x_hat_new);
  P = (I - K * C) * P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

  this->A = A;
  this->dt = dt;
  update(y);
}



// END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS // END OF KALMAN CLASS



unsigned long tnext, tnow;
SoftwareSerial hc06(2, 3);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int currentSign = 0;
int prevSign = 0;
int signChangeCount = 0;
int reps = 0;
imu::Vector<3> vel;
float dt = BNO055_SAMPLERATE_PERIOD_MS / 1000.0;
float absVelocityDuringRep = 0;
float avgConVel = 0;
float avgVel = 0;
float rollingVel = 0;
float rollingConVel = 0;



unsigned long lastOutOfRangeTime = 0;
bool calibrating = false;
bool repInitiated = false;
int repTimer = 0;
int repUpTimer = 0;




void setup(void) {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Reps and Velocity Test");
  Serial.println("");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }


  tnext = millis() + BNO055_SAMPLERATE_PERIOD_MS;
}




int main() {
  setup();

  //KALMAN MODEL AND PARAMETER INITIATION
  Eigen::MatrixXd A(2, 2);
  A << 1, dt, 0, 1; // Dynamics matrix

  Eigen::MatrixXd C(1, 2);
  C << 0, 1; // Measurement matrix

  Eigen::MatrixXd Q(2, 2);
  Q << 1e-5, 0, 0, 1e-5; // Process noise covariance

  Eigen::MatrixXd R(1, 1);
  R << 1e-2; // Measurement noise covariance

  Eigen::MatrixXd P(2, 2);
  P << 1, 0, 0, 1; // Estimate error covariance

  // Initialize the Kalman Filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  Eigen::VectorXd x0(2);
  x0 << 0, 0; // Initial state (position and velocity)
  kf.init(0, x0);

  while (1) {

    imu::Quaternion quat = bno.getQuat();
    quat.x() = -quat.x();
    quat.y() = -quat.y();
    quat.z() = -quat.z();

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> acc;

    acc[0] = (1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z())) * linearaccel[0] + (2 * (quat.x() * quat.y() + quat.w() * quat.z())) * linearaccel[1] + (2 * (quat.x() * quat.z() - quat.w() * quat.y())) * linearaccel[2];
    acc[1] = (2 * (quat.x() * quat.y() - quat.w() * quat.z())) * linearaccel[0] + (1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z())) * linearaccel[1] + (2 * (quat.y() * quat.z() + quat.w() * quat.x())) * linearaccel[2];
    acc[2] = (2 * (quat.x() * quat.z() + quat.w() * quat.y())) * linearaccel[0] + (2 * (quat.y() * quat.z() - quat.w() * quat.x())) * linearaccel[1] + (1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y())) * linearaccel[2];

    static imu::Vector<3> avgacc;
    //noise variable
    float avgrate = 0.05;
    static imu::Vector<3> avgvelp;

    for (int n = 0; n < 3; n++) {
      avgacc[n] = avgrate * acc[n] + (1 - avgrate) * avgacc[n];
      vel[n] += BNO055_SAMPLERATE_PERIOD_MS / 1000.0 * (acc[n] - avgacc[n]);
    }

    // Check if velocity is within the calibration range
    if (vel[2] >= -CALIBRATION_RANGE && vel[2] <= CALIBRATION_RANGE) {
      if (!calibrating) {
        lastOutOfRangeTime = millis();
        calibrating = true;
      } else {
        // If velocity remains within range for more than CALIBRATION_DURATION milliseconds, reset it to 0
        if (millis() - lastOutOfRangeTime >= CALIBRATION_DURATION) {
          vel[2] = 0;
          calibrating = false;
        }
      }
    } else {
      calibrating = false;  // Reset calibration if velocity goes out of range
    }






    if (vel[2] > 0.2) {  //detect sign and increment rep timer for significant velocities
      currentSign = 1;
      if (repInitiated) {
        rollingVel = rollingVel + vel[2];
        repTimer++;

        Eigen::VectorXd y(1);
        y << vel[2]; // Measurement vector
        kf.update(y); // Update the Kalman Filter with the new measurement
        Eigen::VectorXd x = kf.state(); // Get the current state estimate
        Serial.print(acc[2]);
        Serial.print("\t");
        Serial.print(vel[2]);
        Serial.print("\t");
        Serial.println(x[0]);
      }
    } else if (vel[2] < -0.2) {
      currentSign = -1;
      if (repInitiated) {
        rollingConVel = rollingConVel + (-vel[2]);
        rollingVel = rollingVel + (-vel[2]);
        repUpTimer++;
        repTimer++;
        Eigen::VectorXd y(1);
        y << vel[2]; // Measurement vector
        kf.update(y); // Update the Kalman Filter with the new measurement
        Eigen::VectorXd x = kf.state(); // Get the current state estimate
        Serial.print(acc[2]);
        Serial.print("\t");
        Serial.print(vel[2]);
        Serial.print("\t");
        Serial.println(x[0]);
      }
    } else {  //if vel[2] = 0
      currentSign = prevSign;
    }


    //only count the downward motion to initiatge a rep
    if (signChangeCount == 0 && vel[2] < -0.2) {
    } else if (currentSign != prevSign && prevSign != 0) {
      repInitiated = true;
      signChangeCount++;
      prevSign = currentSign;
    }

    prevSign = currentSign;

    if (signChangeCount == 2 && repTimer > 70 && vel[2] < 0.2 && vel[2] > -0.2) {  //we check if enough time has passed for the rep to count
      reps++;
      avgVel = rollingVel / repTimer;
      avgConVel = rollingConVel / repUpTimer;
      signChangeCount = 0;
      repInitiated = false;
      rollingVel = 0;
      rollingConVel = 0;
      repTimer = 0;
      repUpTimer = 0;
      Serial.println("rep: " + String(reps));
      Serial.println("average vel: " + String(avgVel) + "  average concentric vel:" + String(avgConVel));
      hc06.print("rep: " + String(reps));
      hc06.print("average vel: " + String(avgVel) + "  average concentric vel:" + String(avgConVel));
      Serial1.println("average vel: " + String(avgVel) + "  average concentric vel:" + String(avgConVel));
      x0 << 0, 0; // Initial state (position and velocity)
      kf.init(0, x0);

    } else if (signChangeCount == 2 && repTimer <= 70) {  //if a rep is detected in little time, reset
      signChangeCount = 0;
      repInitiated = false;
      rollingVel = 0;
      rollingConVel = 0;
      repTimer = 0;
      repUpTimer = 0;
      // Serial.println("STOP SHAKING.");
      x0 << 0, 0; // Initial state (position and velocity)
      kf.init(0, x0);
    }
  }
}

