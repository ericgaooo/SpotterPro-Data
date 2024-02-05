#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>


#define BNO055_SAMPLERATE_PERIOD_MS 10
#define VELOCITY_THRESHOLD 0.2 // Adjust this threshold for velocity calcs
#define CALIBRATION_RANGE 0.1 // Range for calibration
#define CALIBRATION_DURATION 600 // Duration in milliseconds for calibration


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
float concVelocity = 0;
float prev_vel2 = 0; // Declare prev_vel2 as a global variable




unsigned long lastOutOfRangeTime = 0;
bool calibrating = false;


void setup(void) {
 Serial.begin(115200);
 hc06.begin(9600);
 Serial.println("Reps and Velocity Test");
 Serial.println("");


 if (!bno.begin()) {
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while (1);
 }


 tnext = millis() + BNO055_SAMPLERATE_PERIOD_MS;
}




void loop(void) {
 imu::Quaternion quat = bno.getQuat();
 quat.x() = -quat.x();
 quat.y() = -quat.y();
 quat.z() = -quat.z();


 imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 imu::Vector<3> acc;


 acc[0] = (1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z())) * linearaccel[0] + (2 * (quat.x() * quat.y() + quat.w() * quat.z())) * linearaccel[1] + (2 * (quat.x() * quat.z() - quat.w() * quat.y())) * linearaccel[2];
 acc[1] = (2 * (quat.x() * quat.y() - quat.w() * quat.z())) * linearaccel[0] + (1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z())) * linearaccel[1] + (2 * (quat.y() * quat.z() + quat.w() * quat.x())) * linearaccel[2];
 acc[2] = (2 * (quat.x() * quat.z() + quat.w() * quat.y())) * linearaccel[0] + (2 * (quat.y() * quat.z() - quat.w() * quat.x())) * linearaccel[1] + (1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y())) * linearaccel[2];


 static imu::Vector<3> avgacc;
 //noise variable
 float avgrate = 0.05;


 for (int n = 0; n < 3; n++) {
   avgacc[n] = avgrate * acc[n] + (1 - avgrate) * avgacc[n];
   vel[n] += BNO055_SAMPLERATE_PERIOD_MS / 1000.0 * (acc[n] - avgacc[n]);
 }


 //Serial.println("Zvelocity=");
 //Serial.println(vel(2));
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
   calibrating = false; // Reset calibration if velocity goes out of range
 }


     // int currentSign = (vel[2]>0.1)? 1 : ((vel[2]< -0.1)? -1:0);


     if (vel[2] > 0.1) {
       currentSign = 1;
     }
     else if (vel[2] < -0.1) {
       currentSign = -1;
     }
     else { //if vel[2] = 0
       currentSign = prevSign;
     }


     //only count the downward motion to initiatge a rep
     if (signChangeCount == 0 && vel[2] < -0.1) {
     }
     else if (currentSign != prevSign && prevSign != 0)
     {
       signChangeCount++;
       prevSign = currentSign;
       }


     //Serial.println(signChangeCount);
    


   prevSign = currentSign;


       if (signChangeCount == 2) {
         reps++;
         signChangeCount = 0;
         Serial.println(reps);
       }


}
