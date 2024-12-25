#define __STM32F1__ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

TwoWire Wire2(PB11, PB10); // Define I2C2 with appropriate pins
MPU6050 mpu(0x68, &Wire2); // Use I2C2 for MPU6050

#define OUTPUT_READABLE_YAWPITCHROLL
int const INTERRUPT_PIN = PB1;  // Define the interruption #0 pin
bool blinkState;

// MPU6050 Control/Status Variables
bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// Orientation/Motion Variables
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];
float ypr_mpu[3];

float lastYaw = 0, lastPitch = 0, lastRoll = 0; // Track last angles

volatile bool MPUInterrupt = false;

void DMPDataReady() {
    MPUInterrupt = true;
}

#include "ros.h"
#include "std_msgs/Float32MultiArray.h"

ros::NodeHandle nh;
std_msgs::Float32MultiArray ypr_mpu_msg;
ros::Publisher ypr_mpu_pub("mpu_readings", &ypr_mpu_msg);

void setup() {
    nh.initNode();
    nh.advertise(ypr_mpu_pub);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT);

    Wire2.begin(); // Initialize I2C2

    Serial.begin(9600);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false) {
        Serial.println("MPU6050 connection failed");
        while (true);
    } else {
        Serial.println("MPU6050 connection successful");
    }

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    if (!DMPReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert radians to degrees
        float currentYaw = ypr[0] * 180 / M_PI;
        float currentPitch = ypr[1] * 180 / M_PI;
        float currentRoll = ypr[2] * 180 / M_PI;


            ypr_mpu_msg.data_length = 3;
            ypr_mpu_msg.data[0] = currentYaw;
            ypr_mpu_msg.data[1] = currentPitch;
            ypr_mpu_msg.data[2] = currentRoll;

            ypr_mpu_pub.publish(&ypr_mpu_msg);

            Serial.print("MPU Yaw: ");
            Serial.print(currentYaw);
            Serial.print("\t\tMPU Pitch: ");
            Serial.print(currentPitch);
            Serial.print("\t\tMPU Roll: ");
            Serial.println(currentRoll);

            // Update the last values
            lastYaw = currentYaw;
            lastPitch = currentPitch;
            lastRoll = currentRoll;

            blinkState = !blinkState;
            digitalWrite(LED_BUILTIN, blinkState);
        
    }
}
