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


// Calibration and Tuning Variables
unsigned long lastOutOfRangeTime = 0;
bool calibrating = false;
unsigned long lastOutOfRangeTimeRep = 0;
bool calibratingRep = false;
bool repInitiated = false;
int repTimer = 0;
int repUpTimer = 0;
#define REP_TIMER_MIN_THRESHOLD 70 // Adjustable: Sets minimum amount of time needed for rep to count. 
#define REP_TIMER_MAX_THRESHOLD 500 // Adjustable: Sets max amount of time after rep is initiated before rep is reset. NEEDS TO BE IMPLEMENTED
#define BNO055_SAMPLERATE_PERIOD_MS 10 // Adjustable: Period between each sample from BNO
#define VELOCITY_THRESHOLD 0.2    // Adjustable: 1. downward vel > threshold initiates rep  2. |vel| > |threshold| we use for avg vel 3. |vel| < |threshold| for a rep to end
#define CALIBRATION_RANGE 0.1     // Adjustable: Range for calibration
#define CALIBRATION_DURATION 600  // Adjustable: Duration in milliseconds for calibration
#define LPF_AVG_RATE 0.017    //Adjustable: 0 means weak LPF. 1 means heavy LPF. 
#define ZTILT_THRESHOLD 120 //Adjustable: Greater value = more lenient to tilting 
// Measurement noise covariance

//Sensor related variables
unsigned long tnext, tnow;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> vel;
float dt = BNO055_SAMPLERATE_PERIOD_MS / 1000.0;


//Repetition tracking variables
int currentSign = 0;
int prevSign = 0;
int signChangeCount = 0;


//Derived metric variables
int reps = 0;
float absVelocityDuringRep = 0;
float avgConVel = 0;
float avgVel = 0;
float rollingVel = 0;
float rollingConVel = 0;
float zTiltInitial = 0;
float zTiltDifference = 0;
bool isTilted = false;

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("Serial and BLE monitor test");

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
  Q << 1e-4, 0, 0, 1e-4; // Process noise covariance

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

    //    TEST STATEMENTS
    //    Serial1.println("Hello");
    //    Serial.println("Hello");
    //    delay(2000);


    // START READ FROM SENSOR READ FROM SENSOR READ FROM SENSOR
    imu::Quaternion quat = bno.getQuat();
    quat.x() = -quat.x();
    quat.y() = -quat.y();
    quat.z() = -quat.z();

    sensors_event_t event;
    bno.getEvent(&event);

    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //    float zTiltRaw = event.orientation.z;
    //    Serial.println(zTiltRaw);
    // END READ FROM SENSOR READ FROM SENSOR READ FROM SENSOR


    // START TILT CORRECTION TILT CORRECTION TILT CORRECTION
    imu::Vector<3> acc;
    acc[0] = (1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z())) * linearaccel[0] + (2 * (quat.x() * quat.y() + quat.w() * quat.z())) * linearaccel[1] + (2 * (quat.x() * quat.z() - quat.w() * quat.y())) * linearaccel[2];
    acc[1] = (2 * (quat.x() * quat.y() - quat.w() * quat.z())) * linearaccel[0] + (1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z())) * linearaccel[1] + (2 * (quat.y() * quat.z() + quat.w() * quat.x())) * linearaccel[2];
    acc[2] = (2 * (quat.x() * quat.z() + quat.w() * quat.y())) * linearaccel[0] + (2 * (quat.y() * quat.z() - quat.w() * quat.x())) * linearaccel[1] + (1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y())) * linearaccel[2];
    // END TILT CORRECTION TILT CORRECTION TILT CORRECTION


    //MOVE THIS DECLARATION
    static imu::Vector<3> avgacc;

    // START LPF+EULER LPF+EULER LPF+EULER
    for (int n = 0; n < 3; n++) {
      avgacc[n] = LPF_AVG_RATE * acc[n] + (1 - LPF_AVG_RATE) * avgacc[n];
      vel[n] += BNO055_SAMPLERATE_PERIOD_MS / 1000.0 * (acc[n] - avgacc[n]);
    }
    // END LPF+EULER LPF+EULER LPF+EULER



    // START VEL CALIBRATION CALIBRATION CALIBRATION
    if (vel[2] >= -CALIBRATION_RANGE && vel[2] <= CALIBRATION_RANGE) {
      if (!calibrating) {
        lastOutOfRangeTime = millis();
        calibrating = true;
      } else {
        // If velocity remains within range for more than CALIBRATION_DURATION milliseconds, reset it to 0
        if (millis() - lastOutOfRangeTime >= CALIBRATION_DURATION) {
          //          vel[2] = 0;  //ONLY RESETTING ONE DIMENSION HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!
          vel = {0, 0, 0};
          calibrating = false;
        }
      }
    } else {
      calibrating = false;  // Reset calibration if velocity goes out of range
    }
    // END VEL CALIBRATION CALIBRATION CALIBRATION

    // START REP CALIBRATION REP CALIBRATION REP CALIBRATION
    if (vel[2] >= -0.3 && vel[2] <= 0.3) {
      if (!calibratingRep) {
        lastOutOfRangeTimeRep = millis();
        calibratingRep = true;
      } else {
        // If velocity remains within range for more than CALIBRATION_DURATION milliseconds, reset it to 0
        if (millis() - lastOutOfRangeTimeRep >= 10000) {
          //          vel[2] = 0;  //ONLY RESETTING ONE DIMENSION HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!
          reps = 0;
          calibratingRep = false;
          Serial.println("Reset Reps");
        }
      }
    } else {
      calibratingRep = false;  // Reset calibration if velocity goes out of range
    }
    // END REP CALIBRATION CALIBRATION CALIBRATION


    // START SIGN DETECTION AND REP TIMER+METRIC UPDATING SIGN DETECTION AND REP TIMER+METRIC UPDATING SIGN DETECTION AND REP TIMER+METRIC UPDATING
    if (vel[2] > VELOCITY_THRESHOLD) {  //detect sign and increment rep timer for significant velocities
      currentSign = 1;
      if (repInitiated) {

        //START TILT DETECTION TILT DETECTION TILT DETECTION
        if (repTimer == 0) {
          zTiltInitial = event.orientation.z;
        }
        if (((event.orientation.z >= 15 && event.orientation.z <= 165) || (event.orientation.z >= -165 && event.orientation.z <= -15)) && !isTilted) {
          //          Serial.println(zTiltInitial);
          //          Serial.println(event.orientation.z);
          isTilted = true;
        }
        //END TILT DETECTION TILT DETECTION TILT DETECTION TILT DETECTION

        rollingVel = rollingVel + vel[2];
        repTimer++;
        Eigen::VectorXd y(1);
        y << vel[2]; // Measurement vector
        kf.update(y); // Update the Kalman Filter with the new measurement
        Eigen::VectorXd x = kf.state(); // Get the current state estimate
        //              Serial.print(acc[2]);
        //                Serial.print("\t");
        //                Serial.print(vel[2]);
        //                Serial.print("\t");
        //                Serial.println(x[0]/3);
      }
    } else if (vel[2] < -VELOCITY_THRESHOLD) {
      currentSign = -1;
      if (repInitiated) {
        //START TILT DETECTION TILT DETECTION TILT DETECTION
        if (((event.orientation.z >= 15 && event.orientation.z <= 165) || (event.orientation.z >= -165 && event.orientation.z <= -15)) && !isTilted) {
          //          Serial.println(zTiltInitial);
          //          Serial.println(event.orientation.z);
          isTilted = true;
        }
        //END TILT DETECTION TILT DETECTION TILT DETECTION TILT DETECTION

        rollingConVel = rollingConVel + (-vel[2]);
        rollingVel = rollingVel + (-vel[2]);
        repUpTimer++;
        repTimer++;
        Eigen::VectorXd y(1);
        y << vel[2]; // Measurement vector
        kf.update(y); // Update the Kalman Filter with the new measurement
        Eigen::VectorXd x = kf.state(); // Get the current state estimate
        //                Serial.print(acc[2]);
        //                Serial.print("\t");
        //                Serial.print(vel[2]);
        //                Serial.print("\t");
        //                Serial.println(x[0]/3);
      }
    } else {  //if vel[2] = 0
      currentSign = prevSign;
    }
    // END SIGN DETECTION AND REP TIMER+METRIC UPDATING SIGN DETECTION AND REP TIMER+METRIC UPDATING SIGN DETECTION AND REP TIMER+METRIC UPDATING



    // START DOWNWARD REP INITATION DOWNWARD REP INITATION DOWNWARD REP INITATION DOWNWARD REP INITATION
    if (signChangeCount == 0 && vel[2] < -VELOCITY_THRESHOLD) {
    } else if (currentSign != prevSign && prevSign != 0) {
      repInitiated = true;
      signChangeCount++;
      prevSign = currentSign;
    }
    // END DOWNWARD REP INITATION DOWNWARD REP INITATION DOWNWARD REP INITATION DOWNWARD REP INITATION


    prevSign = currentSign;

    if (signChangeCount == 2 && reps == 0 && repTimer < 300 && vel[2] < VELOCITY_THRESHOLD && vel[2] > -VELOCITY_THRESHOLD) {
      signChangeCount = 0;
      repInitiated = false;
      rollingVel = 0;
      rollingConVel = 0;
      repTimer = 0;
      repUpTimer = 0;
      // Serial.println("STOP SHAKING.");
      x0 << 0, 0; // Initial state (position and velocity)
      kf.init(0, x0);
      isTilted = false;
      continue;
    }
    // START REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK
    if (signChangeCount == 2 && repTimer > REP_TIMER_MIN_THRESHOLD && vel[2] < VELOCITY_THRESHOLD && vel[2] > -VELOCITY_THRESHOLD) {  //we check if enough time has passed for the rep to count
      reps++;
      avgVel = rollingVel / repTimer;
      avgConVel = rollingConVel / repUpTimer;
      signChangeCount = 0;
      repInitiated = false;
      rollingVel = 0;
      rollingConVel = 0;
      repTimer = 0;
      repUpTimer = 0;
      //      Serial.println("rep: " + String(reps));
      Serial.println("rep: " + String(reps) + "average vel: " + String(avgVel) + "  average concentric vel:" + String(avgConVel) + " tilted:" + String(isTilted));
      Serial1.println("r" + String(reps) + "v:" + String(avgVel) + "cv:" + String(avgConVel) + "t:" + String(isTilted));
      x0 << 0, 0; // Initial state (position and velocity)
      kf.init(0, x0);
      isTilted = false;
      // END REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK REP CHECK


      // START SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION
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
      isTilted = false;
    }
    // END SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION SHAKE DETECTION
  }
}

