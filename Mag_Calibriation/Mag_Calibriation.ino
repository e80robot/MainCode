// Magnetometer calibration code

#include <Arduino.h>
#include <Wire.h>
#include "LSM303CTypes.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"

#define READ_BUFFER 100
#define BUFFER_SIZE 100 // number of samples to report to matlab
#define NUM_BYTES_PER_SAMPLE 12 // size of each sample in bytes
#define LED_PIN 13 // GPS Lock LED pin

// variables for data transmission
int read_index;
byte request[READ_BUFFER]; // stores revieved data from matlab
int request_size;
byte message[NUM_BYTES_PER_SAMPLE*BUFFER_SIZE];


float mx,my,mz;
byte *MX,*MY,*MZ;

// intitialize LSM303C object
LSM303C myIMU;


void setup() {
    pinMode(LED_PIN,OUTPUT);
    Serial.begin(115200);
    //Serial.print("Initializing IMU... ");

    if(myIMU.begin(
        ///// Interface mode options
            MODE_I2C,
        ///// Magnetometer output data rate options
            MAG_DO_80_Hz, // 40,20,10,5,2.5,1.25,0,625
        ///// Magnetic field full scale options
            MAG_FS_16_Ga, // 12,8,4
        ///// Magnetometer block data updating options
            MAG_BDU_ENABLE, // MAG_BDU_DISABLE
        ///// Magnetometer X/Y axes ouput data rate
            MAG_OMXY_ULTRA_HIGH_PERFORMANCE, // MAG_OMXY_HIGH_PERFORMANCE
        ///// Magnetometer Z axis ouput data rate
            MAG_OMZ_ULTRA_HIGH_PERFORMANCE, // MAG_OMZ_HIGH_PERFORMANCE
        ///// Magnetometer run mode
            MAG_MD_CONTINUOUS,
        ///// Acceleration full scale
            ACC_FS_2g, // 4,8
        ///// Accelerometer block data updating
            ACC_BDU_ENABLE, // ACC_BDU_DISABLE
        ///// Enable X, Y, and/or Z axis
            ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
        ///// Accelerometer output data rate
            ACC_ODR_100_Hz // 800,400,200,50,10
        ) != IMU_SUCCESS)
    {
        //Serial.println("Initialized unsucessfully");
        // Blink the GPS LOCK LED three times to indicate that the imu did not initialize
        for(int i = 0; i < 3; i++){
            digitalWrite(LED_PIN,HIGH);
            delay(500);
            digitalWrite(LED_PIN,LOW);
            delay(500);
        }
    }
    else{
        //Serial.println("Initialized Sucessfully");
        digitalWrite(LED_PIN,HIGH);
    }
    
  delay(2000); // Wait for Serial communication to stabalize
}


void loop() {
    
    read_index = 0;
    request_size = 0;
    while(!Serial.available()){}// Wait for Matlab to request data
    while(Serial.available()){ // Read Matlab buffer size [bytes]
        // Bytes are recieved from lsd to msd. Each byte encodes a single base-10 digit
        request[read_index] = Serial.read(); // Expect an ASCII number null terminated
        read_index++;
    }
    for(int i=0; i<read_index; i++){
        // Convert request buffer to an integer value stored in 'request_size'
        request_size += 10^i*request[i];
    }
    //delay(100); // Wait before collecting data

    for(int i=0; i<BUFFER_SIZE; i++){
        // get current data from IMU in IEEE 754 format floats        
        mx = myIMU.readMagX();
        my = myIMU.readMagY();
        mz = myIMU.readMagZ();
        
        // convert floats to bytes for serial transmission
        MX = (byte*) & mx;
        MY = (byte*) & my;
        MZ = (byte*) & mz;

        // place current data in message buffer
        message[NUM_BYTES_PER_SAMPLE*i]   = MX[0];
        message[NUM_BYTES_PER_SAMPLE*i+1] = MX[1];
        message[NUM_BYTES_PER_SAMPLE*i+2] = MX[2];
        message[NUM_BYTES_PER_SAMPLE*i+3] = MX[3];

        message[NUM_BYTES_PER_SAMPLE*i+4] = MY[0];
        message[NUM_BYTES_PER_SAMPLE*i+5] = MY[1];
        message[NUM_BYTES_PER_SAMPLE*i+6] = MY[2];
        message[NUM_BYTES_PER_SAMPLE*i+7] = MY[3];

        message[NUM_BYTES_PER_SAMPLE*i+8]  = MZ[0];
        message[NUM_BYTES_PER_SAMPLE*i+9]  = MZ[1];
        message[NUM_BYTES_PER_SAMPLE*i+10] = MZ[2];
        message[NUM_BYTES_PER_SAMPLE*i+11] = MZ[3];

        // this delay prevents repeat sampling
        delay(8); 
    }
    Serial.write(message,sizeof(message));
}

