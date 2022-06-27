/*
 * Louis Kraft
 * EEE199 - Navigation Control Program
 * 
*/

// --- That's a lot of libraries ---
#include <ams_as5048b.h>
#include <Wire.h>
#include <analogWrite.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>
#include <SharpDistSensor.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>

// --- Bluetooth
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- Robot Parameters
#define MOUSE_R 17.25 // Radius of each drive wheel (mm). 
#define MOUSE_L 42   // Distance from the center of robot to center of wheel(mm)

// --- Motor PWM_PINS: PWM_x0 HIGH, PWM_x1 LOW -> forward direction
#define PWM_L0 18
#define PWM_L1 5
#define PWM_R0 4
#define PWM_R1 2

// --- Analog IR distance signal PIN
#define DIST_PIN_L2 14
#define DIST_PIN_L1 27
#define DIST_PIN_C0 26
#define DIST_PIN_R1 25
#define DIST_PIN_R2 33

// --- Range sensors analog output pins
const byte nbSensors = 5;
const byte medianFilterWindowSize = 5;
SharpDistSensor sensorArray[] = {
  SharpDistSensor(DIST_PIN_L2, medianFilterWindowSize), 
  SharpDistSensor(DIST_PIN_L1, medianFilterWindowSize),
  SharpDistSensor(DIST_PIN_C0, medianFilterWindowSize), 
  SharpDistSensor(DIST_PIN_R1, medianFilterWindowSize),
  SharpDistSensor(DIST_PIN_R2, medianFilterWindowSize), 
};

// Measured distance array (mm)
static double distArray[nbSensors];

// --- Wheel encoder 3.3V power pins
const int ENCODER_PIN_LEFT = 16;
const int ENCODER_PIN_RIGHT = 17;

// --- Live data variables
static double (*wheel_velocity);
static int theta = 0; 
static int pose[7] = {0,0,0,0,0,0,0}; //{x_0, y_0, theta_0, x_f, y_f, theta_f, ReadWrite MATLAB}

// --- PID control variables/ Gains
static double pid_input_left, pid_output_left, pid_setpoint_left = 0;
const double pid_gain_left[3] = {90,0,0}; // {Kp, Ki, Kd}
const double pid_gain_left_start[3] = {45,0,0};

static double pid_input_right, pid_output_right, pid_setpoint_right = 0;
const double pid_gain_right[3] = {90,0,0}; // {Kp, Ki, Kd}
const double pid_gain_right_start[3] = {45,0,0};

// --- Kalman Filter variables
const double kalman_velocity_gain = 1;
const double kalman_encoder_gain = 0.5;
const double kalman_noise = 0.7;

// --- Object instances
AMS_AS5048B encoder_left(0x40); 
AMS_AS5048B encoder_right(0x80);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // IMU
PID pid_left(&pid_input_left, &pid_output_left, &pid_setpoint_left, pid_gain_left[0], pid_gain_left[1], pid_gain_left[2], DIRECT);
PID pid_right(&pid_input_right, &pid_output_right, &pid_setpoint_right, pid_gain_right[0], pid_gain_right[1], pid_gain_right[2], DIRECT);
SimpleKalmanFilter filterWheelVelocityL(kalman_velocity_gain, kalman_velocity_gain, kalman_noise);
SimpleKalmanFilter filterWheelVelocityR(kalman_velocity_gain, kalman_velocity_gain, kalman_noise);
SimpleKalmanFilter filterEncoder(kalman_encoder_gain, kalman_encoder_gain, kalman_noise);

// --- Functions forward declarations
void setWheelVelocity(double output_L, double output_R);
void encoderInit(void);
void motorStop(void);
void readDistanceArray(void);
double (*(getForceVelocity)(double distance_array[5]));
double (*(setPosition)(double distance));
double (*(initWheelVelocity)(void));
double (*(getWheelVelocity)(void));
double forceRep(double dist);
double (*(setTheta)(void));
double calculateDistance(void);
int calculateTheta(void);
int readTheta(void);

// --- BLE Characteristic Class Read/Write 
class MyCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      std::string value = pCharacteristic->getValue();
        
      std::string s = value;
      std::string delimiter = ".";
      size_t pos = 0;
      std::string token; 

      int i = 0;
      while ((pos = s.find(delimiter)) != std::string::npos)
      {
        token = s.substr(0, pos);
        const char *cstr = token.c_str();
        String ccc = String(cstr);
        s.erase(0, pos + delimiter.length());          
        pose[i] = ccc.toInt();
        i++;
      }
    }

    void onRead(BLECharacteristic *pCharacteristic)
    { 
        std::ostringstream temp;
        for(int i:pose)
        {
          temp << i << "." ; 
        }
        std::string str(temp.str());
        pCharacteristic->setValue(str); 
    }
    
};

// --------------------------------------------------
// ------------------ ESP32 SETUP -------------------
// --------------------------------------------------
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  // Motor Diver Pins
  analogWriteResolution(PWM_L0, 10);
  analogWriteResolution(PWM_L1, 10);
  analogWriteResolution(PWM_R0, 10);
  analogWriteResolution(PWM_R1, 10);

  // Distance sensor configuration
  analogReadResolution(10);
  for (byte i = 0; i < nbSensors; i++) {
    sensorArray[i].setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);  // Set sensor model
  }

  // PID left wheel parameters
  pid_left.SetMode(AUTOMATIC);
  pid_left.SetSampleTime(5);
  pid_left.SetOutputLimits(0, 1024);
  
  // PID right wheel parameters
  pid_right.SetMode(AUTOMATIC);
  pid_right.SetSampleTime(5);
  pid_right.SetOutputLimits(0, 1024);
  
  wheel_velocity = initWheelVelocity();
  pid_input_left = wheel_velocity[0];
  pid_input_right = wheel_velocity[1];
  
  encoderInit();
  motorStop(); 

  // Initialize IMU
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //BLE Setup
  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

// ----------------------------------------------------
// -------------------------- MAIN LOOP ---------------
// ----------------------------------------------------
void loop()
{ 
  static byte goal_reached = 0;
  static double *wheel_velocity; 
  static double *force_velocity;
  static int forward_move_distance;

  // initialize distArray
  for(int i=0; i<3; i++)
  {
    readDistanceArray();
  }

  // Path following Algorithm
  while(!goal_reached)
  {
    Serial.println("GOAL NOT REACHED");    
    // Sets theta from pose values
    pose[2] = readTheta();
    while( (abs(pose[5]-pose[2]) > 5) && pose[6] == 0)
    {
      Serial.println("SETTING POSE");
      wheel_velocity = setTheta();
      setWheelVelocity(wheel_velocity[0], wheel_velocity[1]);
    }   

    // drive forward, avoid abstacles
    while(pose[6] == 1)
    {
      forward_move_distance = calculateDistance(pose[3], pose[0], pose[4], pose[1]);
      Serial.print("forward move distance: ");
      Serial.println(forward_move_distance);
      readDistanceArray();
      force_velocity = getForceVelocity(distArray); 
      wheel_velocity = setPosition(forward_move_distance);
      setWheelVelocity(wheel_velocity[0] + force_velocity[0], wheel_velocity[1] + force_velocity[1]);
    }
    
    motorStop();
    
    if(pose[6] == 3)
    {
      goal_reached = 1;
    }

        
  } // Goal reached

  goal_reached = 0;
}

// -------------------------------
// ---------- Functions ----------
// -------------------------------

// ----------------------------------------------
// ---------- Obstacle Avoidance ----------------
// ----------------------------------------------
// Updates the distance array. Distance from the robot to detected object.
void readDistanceArray(void)
{
   for (byte i = 0; i < nbSensors; i++) 
   {
    distArray[i] = (double)sensorArray[i].getDist();
   } 
}

// Calculates an new wheel velocity offset using potential field
double (*(getForceVelocity)(double distance_array[5]))
{
    // Calculation is made relative to theta which is 90* relative to L
    static double force_velocity[2] = {0,0};
    double force_array[5] = {0,0,0,0,0};

    for(int i=0; i<5; i++)
    {
      force_array[i] = forceRep(distance_array[i]);
    }

    // Calculate Fx components
    force_velocity[0] = force_array[0] + force_array[1]*0.707106 - force_array[3]*0.707106 - force_array[4];
    
    // Calculate Fy components
    force_velocity[1] = - force_array[1]*0.707106 - force_array[2] - force_array[3]*0.707106;
    
    // Fx moves negative x direction
    if(force_velocity[0] < 0)
    {
      force_velocity[0] =  force_velocity[0] + force_velocity[1];
      force_velocity[1] =  -force_velocity[0];
      return force_velocity;
    }

    // Fx moves positive x direction
    if(force_velocity[0] >= 0)
    {
      force_velocity[0] = force_velocity[0];
      force_velocity[1] = force_velocity[1] - force_velocity[0]; 
      return force_velocity;
    }
}

// Returns repulsive force values from artificial potential field function
double forceRep(double dist)
{   
    // All distance values are in mm
    double dist_minimum = 190; //G0
    double M = 3;
    double k_rep = 20;
    double force_rep1, force_rep2;
    static double force_rep = 0;
    double dist_2obj = dist; //Rr
    double dist_2goal = dist; //Ra
    
    force_rep1 = k_rep*(1/dist_2obj - 1/dist_minimum);
    force_rep2 = M*k_rep*pow((1/dist_2obj - 1/dist_minimum)*(1/dist_2obj - 1/dist_minimum),2)*(pow(dist_2goal, M));
    
    force_rep = force_rep1 + force_rep2;
    
    return force_rep;
}
// ----------------------------------------------
// ---------- Kinematics ------------------------
// ----------------------------------------------

void updatePose(double w, double t_0)
{
  double temp[2] = {0,0};
  pose[2] = readTheta();
  temp[0] = w*t_0*0.001*MOUSE_R*cos((double)pose[2]*(3.14159/180));
  temp[1] = w*t_0*0.001*MOUSE_R*sin((double)pose[2]*(3.14159/180));

  Serial.print("x_0: ");
  Serial.print(pose[0]);
  pose[0] = pose[0] + (int)round(temp[0]);
  Serial.print("     x_f: ");
  Serial.println(pose[0]);
  pose[1] = pose[1] + (int)round(temp[1]);
}

void updatePose2(double w_l, double w_r, double t_0)
{
  double temp[2] = {0,0};
  double omega = (MOUSE_R*(w_r - w_l))/MOUSE_L;
  double icc[2] = {pose[0] - MOUSE_R*sin(pose[2]*(M_PI/180)), pose[1] + MOUSE_R*cos(pose[2]*(M_PI/180))};
  
  //temp[0] = w*t_0*0.001*MOUSE_R*cos((double)pose[2]*(3.14159/180));
  //temp[1] = w*t_0*0.001*MOUSE_R*sin((double)pose[2]*(3.14159/180));

  Serial.print("x_0: ");
  Serial.print(pose[0]);
  pose[0] = pose[0] + (int)round(temp[0]);
  Serial.print("     x_f: ");
  Serial.println(pose[0]);
  pose[1] = pose[1] + (int)round(temp[1]);
  pose[2] = readTheta();
}

// --- (Input) Read wheel Velocity
double (*(getWheelVelocity)(void))
{
  static double wv[2] = {0,0};
  static double angle[2] = {0,0};
  static unsigned long t1,t2 = 0;
  static double dt = 0;

  t1 = micros();
  angle[0] = encoder_left.getMovingAvgExp(U_RAD);
  angle[1] = encoder_right.getMovingAvgExp(U_RAD);
  encoder_left.updateMovingAvgExp();
  encoder_right.updateMovingAvgExp();
  
  t2 = micros();
  angle[0] = fabs(encoder_left.getMovingAvgExp(U_RAD) - angle[0]);
  angle[1] = fabs(encoder_right.getMovingAvgExp(U_RAD) - angle[1]);
  encoder_left.updateMovingAvgExp();
  encoder_right.updateMovingAvgExp();

  dt = ((t2-t1)*0.000001); // Convert (us) to (s)
  
  for(int i=0; i<2; i++)
  {
    if(angle[i] >= 1)
    {
      angle[i] = 2*3.14159 - angle[i];  
    } 
    
    wv[i] = angle[i]/dt;

    if(wv[i] < 1.6)
    { 
     wv[i] = 0;
    }

  }

  return wv;
}

// --- (Output) Set Wheel Velocity rad/s---
void setWheelVelocity(double output_L, double output_R)
{
  pid_setpoint_left = fabs(output_L); 
  pid_setpoint_right = fabs(output_R);

  wheel_velocity = getWheelVelocity();

  pid_input_left = filterWheelVelocityL.updateEstimate(wheel_velocity[0]);
  pid_left.Compute();
   
  pid_input_right = filterWheelVelocityR.updateEstimate(wheel_velocity[1]);
  pid_right.Compute();

  // Set the gains depending on the distance from the setpoint, reduces overshoot
  if(pid_input_left < 3)
  {
    pid_left.SetTunings(pid_gain_left_start[0], pid_gain_left_start[1], pid_gain_left_start[2]); 
  }

  if(pid_input_right < 3)
  {
    pid_right.SetTunings(pid_gain_right_start[0], pid_gain_right_start[1], pid_gain_right_start[2]); 
  }

  else
  {
    pid_left.SetTunings(pid_gain_left[0], pid_gain_left[1], pid_gain_left[2]);
    pid_right.SetTunings(pid_gain_right[0], pid_gain_right[1], pid_gain_right[2]);        
  }

  // Forward
  if(output_L >= 0 && output_R >= 0)
  {
    analogWrite(PWM_L1, (int)pid_output_left);
    analogWrite(PWM_L0, 0);  
    analogWrite(PWM_R0, 0);
    analogWrite(PWM_R1, (int)pid_output_right);
    return;
  }
  //Rotate ccw
  if(output_L < 0 && output_R > 0)
  {
    analogWrite(PWM_L1, 0);
    analogWrite(PWM_L0, (int)pid_output_left);  
    analogWrite(PWM_R0, 0);
    analogWrite(PWM_R1, (int)pid_output_right);
    return;
  }
  // Rotate cw
  if(output_L > 0 && output_R < 0)
  {
    analogWrite(PWM_L1, (int)pid_output_left);
    analogWrite(PWM_L0, 0);  
    analogWrite(PWM_R0, (int)pid_output_right);
    analogWrite(PWM_R1, 0);
    return;
  }
  // Reverse
  if(output_L < 0 && output_R < 0)
  {
    analogWrite(PWM_L1, 0);
    analogWrite(PWM_L0, (int)pid_output_left);  
    analogWrite(PWM_R0, (int)pid_output_right);
    analogWrite(PWM_R1, 0);  
    return;
  }
}

// --- Read the IMU for orrientation (theta) angle
int readTheta(void)
{
  static int theta;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  theta = (int)round(euler.x());
  theta = map(theta, 0, 360, 360, 0);
  return theta;  
}

// --- Set theta returns pid_setpoints to rotate cw/ccw 
double *((setTheta)(void))
{
  static double pid_setpoint[2] = {0,0};
  double theta_gain = 0;
  int delta_theta = 0;
  pose[2] = readTheta();
  delta_theta = pose[5] - pose[2];
  
  if(abs(delta_theta) <= 20){
    theta_gain = 0.85;
  }
  else 
  {
     theta_gain = 0.4;
  }

  if((pose[5] >= pose[2]) && abs(delta_theta)>=180)
  {
    pid_setpoint[0] = theta_gain*(360-delta_theta); 
    pid_setpoint[1] = -theta_gain*(360-delta_theta);
    return pid_setpoint;
  }

  if((pose[5] >= pose[2]) && abs(delta_theta)<180)
  {
    pid_setpoint[0] = -theta_gain*delta_theta; 
    pid_setpoint[1] = theta_gain*delta_theta;
    return pid_setpoint;
  }

  if((pose[5] < pose[2]) && abs(delta_theta)>=180)
  {
    pid_setpoint[0] = -1.2*theta_gain*(360 - abs(delta_theta)); 
    pid_setpoint[1] = 1.2*theta_gain*(360 - abs(delta_theta));
    return pid_setpoint;
  }
  
  if((pose[5] < pose[2]) && abs(delta_theta)<180)
  {
    pid_setpoint[0] = 1.2*theta_gain*abs(delta_theta); 
    pid_setpoint[1] = -1.2*theta_gain*abs(delta_theta);
    return pid_setpoint;
  }  
}

// --- Calculate theta for point to point move
int calculateTheta(void)
{
  double point_theta = 0;
  double x = pose[3] - pose[0];
  double y = pose[4] - pose[1];
  
  if(x > 0)
  {
    point_theta = atan(y/x)*57.2957795131;
  }
  if(x < 0 && y >= 0)
  {
    point_theta =180 + atan(y/x)*57.2957795131;
  }
  if(x < 0 && y < 0)
  {
    point_theta = 180 + atan(y/x)*57.2957795131;
  }
  if(x == 0 && y > 0)
  {
    point_theta = 90;
  }
  if(x =0 && y < 0)
  {
    point_theta = -90;
  }
  if(point_theta < 0)
  {
    point_theta = point_theta + 360;  
  }
  return (int)(round(point_theta));
}

// --- Set Position returns pid_setpoints to move robot to new position 
double *((setPosition)(double distance))
{
  static double pid_setpoint[2] = {0,0};
  double position_error = distance;
  double position_gain = 0.45;
  for(int i=0; i<2; i++)
  {
    pid_setpoint[i] = position_gain*position_error;
    
    // Max output for setpoint 15rad/s
    if(pid_setpoint[i] >= 6)
    {
      pid_setpoint[i] = 6;
    }
  }
  return pid_setpoint;
}

// --- Wheel velocity initializer
double (*(initWheelVelocity(void)))
{
  static double wv[2] = {0,0};
  wv[0] = 0;
  wv[1] = 0;
  return wv;
}

// --- Encoder(s) Initializer
void encoderInit(void)
{
  pinMode(ENCODER_PIN_LEFT, OUTPUT);
  pinMode(ENCODER_PIN_RIGHT, OUTPUT);

  digitalWrite(ENCODER_PIN_LEFT,HIGH);
  digitalWrite(ENCODER_PIN_RIGHT,HIGH);
  
  encoder_left.begin();
  encoder_left.setZeroReg();
  
  encoder_right.begin();
  encoder_right.setZeroReg();
  encoder_right.setClockWise(true);
}

// --- Motor stop
void motorStop(void)
{
  analogWrite(PWM_R0, 0);
  analogWrite(PWM_R1, 0);
  analogWrite(PWM_L0, 0);
  analogWrite(PWM_L1, 0);
}

// --- Calculate the distance between two points from pose array
double calculateDistance(void)
{
  static double distance = 0;
  distance = sqrt((pose[3] - pose[0])*(pose[3] - pose[0])+(pose[4] - pose[1])*(pose[4] - pose[1]));
  return distance;
}

// --- Calculate the distance between two points input
double calculateDistance(double x_f, double x_0, double y_f, double y_0)
{
  static double distance = 0;
  distance = sqrt((x_f - x_0)*(x_f - x_0)+(y_f - y_0)*(y_f - y_0));
  return distance;
}