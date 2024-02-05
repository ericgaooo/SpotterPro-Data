#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>

#define BNO055_SAMPLERATE_PERIOD_MS 10
#define VELOCITY_THRESHOLD 0.2   // Adjust this threshold for velocity calcs
#define CALIBRATION_RANGE 0.1    // Range for calibration
#define CALIBRATION_DURATION 600 // Duration in milliseconds for calibration

unsigned long tnext, tnow;

// Variable delarations
SoftwareSerial hc06(2, 3);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
imu::Vector<3> vel;
static imu::Vector<3> avgacc;

int currentSign = 0;
int prevSign = 0;
int signChangeCount = 0; // Used for rep detection
int reps = 0;

float dt = BNO055_SAMPLERATE_PERIOD_MS / 1000.0;
float absVelocityDuringRep = 0;
float concVelocity = 0;
unsigned long lastOutOfRangeTime = 0;
bool calibrating = false;
float avgrate = 0.05; // Noise paramneter

// Connect to sensor
void setup(void)
{
    Serial.begin(115200);
    hc06.begin(9600);
    Serial.println("Reps and Velocity Test");
    Serial.println("");

    if (!bno.begin())
    {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    tnext = millis() + BNO055_SAMPLERATE_PERIOD_MS;
}

// Loop for data from sensor
void loop(void)
{
    // Retrieve and break down quaternion
    imu::Quaternion quat = bno.getQuat();
    quat.x() = -quat.x();
    quat.y() = -quat.y();
    quat.z() = -quat.z();

    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> acc;
    // Get acceleration components
    acc[0] = (1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z())) * linearaccel[0] + (2 * (quat.x() * quat.y() + quat.w() * quat.z())) * linearaccel[1] + (2 * (quat.x() * quat.z() - quat.w() * quat.y())) * linearaccel[2];
    acc[1] = (2 * (quat.x() * quat.y() - quat.w() * quat.z())) * linearaccel[0] + (1 - 2 * (quat.x() * quat.x() + quat.z() * quat.z())) * linearaccel[1] + (2 * (quat.y() * quat.z() + quat.w() * quat.x())) * linearaccel[2];
    acc[2] = (2 * (quat.x() * quat.z() + quat.w() * quat.y())) * linearaccel[0] + (2 * (quat.y() * quat.z() - quat.w() * quat.x())) * linearaccel[1] + (1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y())) * linearaccel[2];

    // Get average acceleration with noise considered
    for (int n = 0; n < 3; n++)
    {
        avgacc[n] = avgrate * acc[n] + (1 - avgrate) * avgacc[n];
        vel[n] += BNO055_SAMPLERATE_PERIOD_MS / 1000.0 * (acc[n] - avgacc[n]);
    }

    // Serial.println("Zvelocity=");
    // Serial.println(vel(2));

    // Checks if velocity is within the calibration range
    if (vel[2] >= -CALIBRATION_RANGE && vel[2] <= CALIBRATION_RANGE)
    {
        if (!calibrating)
        {
            lastOutOfRangeTime = millis();
            calibrating = true;
        }
        else
        {
            // If velocity remains within range for more than CALIBRATION_DURATION milliseconds, reset it to 0
            if (millis() - lastOutOfRangeTime >= CALIBRATION_DURATION)
            {
                vel[2] = 0;
                calibrating = false;
            }
        }
    }
    else
    {
        calibrating = false; // Reset calibration if velocity goes out of range
    }

    // Change the sign if velocity is significant
    if (vel[2] > 0.1)
    {
        currentSign = 1;
    }
    else if (vel[2] < -0.1)
    {
        currentSign = -1;
    }
    else // If vel[2] = 0
    {
        currentSign = prevSign;
    }

    // Increment sign change counter
    if (signChangeCount == 0 && vel[2] < -0.1)
    {
        // Do not increment signChangeCount if a rep has not started and the velocity is moving upwards.
        // Filters out unrack and unintentional movements. Reps are started by significant downwards velocity.
        // This can be switched (e.g. bench is down-up. deadlift is up-down).
    }
    else if (currentSign != prevSign && prevSign != 0) // Increment signChangeCount
    {
        signChangeCount++;
    }
    prevSign = currentSign; // Set prevSign

    if (signChangeCount == 2) // If we have 2 sign changes, a rep has been done. Increment reps, reset signChangeCount, and print.
    {
        reps++;
        signChangeCount = 0;
        Serial.println(reps);
    }
}
