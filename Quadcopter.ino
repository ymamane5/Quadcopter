#include "I2Cdev.h"
#include <helper_3dmath.h>
#include <ESP32_Servo.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED_PIN 2

#define SERIAL_PORT_SPEED 115200
#define RC_NUM_CHANNELS  5

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH9  4

#define RC_CH1_INPUT  35
#define RC_CH2_INPUT  34
#define RC_CH3_INPUT  32
#define RC_CH4_INPUT  33
#define RC_CH9_INPUT  26

#define MOTOR1_PIN_OUTPUT  12 
#define MOTOR2_PIN_OUTPUT  27
#define MOTOR3_PIN_OUTPUT  25
#define MOTOR4_PIN_OUTPUT  13

#define P_GAIN 0.3
#define I_GAIN 0.0
#define D_GAIN 40.0

#define P_GAIN_YAW 2.0
#define I_GAIN_YAW 0.0

#define MAX_FIX 150
#define D_MAX_FIX 200

MPU6050 mpu;
bool blinkState = false;
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
uint16_t rc_final[RC_NUM_CHANNELS];
Servo motor1, motor2, motor3, motor4;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_degree[3];    // [yaw, pitch, roll] in degrees
float ypr_offset[3];    // [yaw, pitch, roll] offset
float ypr_center[3] = {-160.0, -15.0, -2.0};
float ypr_desired[3]; // desired angle
float prev_p_temp_pitch = 0, prev_p_temp_roll = 0, prev_p_temp_yaw = 0, pitch = 0, roll = 0, yaw = 0, pitch_receiver, roll_receiver, yaw_receiver;
float p_pitch = 0, p_roll = 0, p_yaw = 0, i_pitch = 0, i_roll = 0, i_yaw = 0,  d_pitch = 0, d_roll = 0;

int test_pin;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

uint8_t counter = 0;
bool gyroInitialized = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    rc_shared[channel] = (uint16_t)(micros() - rc_start[channel]);
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch9() { calc_input(RC_CH9, RC_CH9_INPUT); }

void setup() {
  
    pinMode(RC_CH1_INPUT, INPUT);
    pinMode(RC_CH2_INPUT, INPUT);
    pinMode(RC_CH3_INPUT, INPUT);
    pinMode(RC_CH4_INPUT, INPUT);
    pinMode(RC_CH9_INPUT, INPUT);
  
    motor1.attach(MOTOR1_PIN_OUTPUT, 1100, 2000);
    motor2.attach(MOTOR2_PIN_OUTPUT, 1100, 2000);
    motor3.attach(MOTOR3_PIN_OUTPUT, 1100, 2000);
    motor4.attach(MOTOR4_PIN_OUTPUT, 1100, 2000);
  
    attachInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE); //for arduino - enableInterrupt
    attachInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
    attachInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
    attachInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
    attachInterrupt(RC_CH9_INPUT, calc_ch9, CHANGE);

    Serial.begin(SERIAL_PORT_SPEED);

//-----------------------------Gyro----------------------------------
  
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(21 , 22, 400000); // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(100);
    mpu.setYGyroOffset(300);
    mpu.setZGyroOffset(-350);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (ESP32 pin 32)..."));
        pinMode(4, INPUT_PULLUP);
        attachInterrupt(4, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

//------------------------------Main program loop---------------------------------------

void loop() {
  
  float p_temp_pitch, p_temp_roll, p_temp_yaw;
  
  rc_read_values();
  rc_final[RC_CH1] = rc_final[RC_CH2] = rc_final[RC_CH3] = rc_final[RC_CH4] = rc_values[RC_CH3];
  
  if(gyroInitialized && !(rc_values[RC_CH9] & 0x300)) 
  {
	if(rc_values[RC_CH3] > 1200)  // throttle is not min
    {
        pitch_receiver = (rc_values[RC_CH2] - 1531);
        roll_receiver = (rc_values[RC_CH1] - 1546);
        yaw_receiver = rc_values[RC_CH4] - 1526;
		
		if(pitch_receiver < 4 && pitch_receiver > -4)
			pitch_receiver = 0;
		if(roll_receiver < 4 && roll_receiver > -4)
			roll_receiver = 0;
		if(yaw_receiver < 4 && yaw_receiver > -4)
			yaw_receiver = 0;
		
       
	   // --------calculate P--------
	   
		//	    pitch receiver:	-40->40		 - 		gyro: -45->45
		p_temp_pitch = 	((float)pitch_receiver * 0.1) - ypr_offset[1];
		p_pitch = p_temp_pitch * P_GAIN;
		if(p_pitch > MAX_FIX)
            p_pitch = MAX_FIX;
		else if(p_pitch < -MAX_FIX)
            p_pitch = -MAX_FIX;
		
		//   	roll receiver:	-40->40	 	- 		gyro: -45->45
		p_temp_roll = ((float)roll_receiver * 0.1) - ypr_offset[2];
		p_roll = p_temp_roll * P_GAIN;
		if(p_roll > MAX_FIX)
            p_roll = MAX_FIX;
		else if(p_roll < -MAX_FIX)
            p_roll = -MAX_FIX;
        
		p_temp_yaw = (yaw_receiver * 0.4) - ypr_offset[0];
		p_yaw = p_temp_yaw * P_GAIN_YAW;
		if(p_yaw > MAX_FIX)
            p_yaw = MAX_FIX;
		else if(p_yaw < -MAX_FIX)
            p_yaw = -MAX_FIX;
		
		// --------calculate I---------
		
		if(i_pitch > 100) 
			i_pitch = 100;
		else if(i_pitch < -100) 
			i_pitch = -100;
		else
			i_pitch += p_temp_pitch * I_GAIN;
		
		if(i_roll > 100) 
			i_roll = 100;
		else if(i_roll < -100) 
			i_roll = -100;
		else
			i_roll += p_temp_roll * I_GAIN;
		
		if(i_yaw > 100) 
			i_yaw = 100;
		else if(i_yaw < -100) 
			i_yaw = -100;
		else
			i_yaw += p_temp_yaw * I_GAIN_YAW;
		
		// ----------calculate D-----------
		
		d_pitch = p_temp_pitch - prev_p_temp_pitch;
		d_pitch *= D_GAIN;
		if(d_pitch > D_MAX_FIX)
            d_pitch = D_MAX_FIX;
		else if(d_pitch < -D_MAX_FIX)
            d_pitch = -D_MAX_FIX;
		
		d_roll = p_temp_roll - prev_p_temp_roll;
		d_roll *= D_GAIN;
		if(d_roll > D_MAX_FIX)
            d_roll = D_MAX_FIX;
		else if(d_roll < -D_MAX_FIX)
            d_roll = -D_MAX_FIX;

		// *********************************************************
		
		pitch = p_pitch + i_pitch + d_pitch;
		roll =  p_roll  + i_roll +  d_roll ;
		yaw =   p_yaw   + i_yaw;
		
		prev_p_temp_pitch = p_temp_pitch;
		prev_p_temp_roll = p_temp_roll;
		prev_p_temp_yaw = p_temp_yaw;
		
    }
	else {// throttle is min
		yaw = pitch = roll = 0;
	}
    rc_final[RC_CH1] = rc_final[RC_CH1] + pitch - roll - yaw;
    rc_final[RC_CH2] = rc_final[RC_CH2] + pitch + roll + yaw;
    rc_final[RC_CH3] = rc_final[RC_CH3] - pitch + roll - yaw;
    rc_final[RC_CH4] = rc_final[RC_CH4] - pitch - roll + yaw;
	
  }
  else
      rc_final[RC_CH1] = rc_final[RC_CH2] = rc_final[RC_CH3] = rc_final[RC_CH4] = 1000;
 
  motor1.writeMicroseconds( rc_final[RC_CH1]);
  motor2.writeMicroseconds(rc_final[RC_CH2]);
  motor3.writeMicroseconds(rc_final[RC_CH3]);  
  motor4.writeMicroseconds(rc_final[RC_CH4]);
 
  //------------------------------Gyro------------------------------------

  if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr_degree, ypr_offset, &q, &gravity, ypr_center);
        if(counter == 0 && !gyroInitialized)
            gyroInitialized = mpu.initializeGyroAxis(ypr_center, ypr_degree);
            

        if(gyroInitialized) {   
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr_degree, ypr_offset, &q, &gravity, ypr_center);
		    Serial.print("yaw_offset\t");
            Serial.print(ypr_offset[0]);
			Serial.print("\tpitch_offset\t");
            Serial.print(ypr_offset[1]);
			Serial.print("\troll_offset\t");
            Serial.println(ypr_offset[2]);
       }
        counter++;

        digitalWrite(LED_PIN, !gyroInitialized);
        
    }

} 
