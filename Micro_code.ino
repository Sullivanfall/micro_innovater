#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create instances for the 5 VL53L0X sensors
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor5 = Adafruit_VL53L0X();

// Define XSHUT pins for the 5 VL53L0X sensors and Gyroscope
#define XSHUT_1 13
#define XSHUT_2 26
#define XSHUT_3 25 
#define XSHUT_4 33
#define XSHUT_5 32 
// I2C Pins
#define SDA_PIN 14
#define SCL_PIN 27

// GYROSCOPE CODE
#include <MPU9250_asukiaaa.h>
#include <cmath>

MPU9250_asukiaaa sensor6;

//Calibration variables
float gyro_x_offset = 0.0, gyro_y_offset = 0.0, gyro_z_offset = 0.0;
float gyro_z_angle = 0.0; // Integrated angle fren gyroscope

unsigned long previousTime=0;

// m1 is for right motor
// m2 is for left motor
#define ENCA_m1 16
#define ENCB_m1 17
#define PWM_m1 23
#define IN2_m1 22
#define IN1_m1 21

#define ENCA_m2 18
#define ENCB_m2 19
#define PWM_m2 15
#define IN2_m2 2
#define IN1_m2 4

#define standby 5

int pos_m1 = 0;
int pos_m2 = 0;
int speedMotor = 25;
// the value will be based on the number of pulses in one revolution.
// need to set the value of cell_distance (distance between cells)
// need to set the value of turn_distance (how much to turn)
int cell_distance, turn_distance;


void setup() {
  // Start Serial for debugging
  Serial.begin(115200);
  delay(100);

  // Initialize I2C communication
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize XSHUT pins
  pinMode(XSHUT_1, OUTPUT); pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT); pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  // Turn off both sensors
  digitalWrite(XSHUT_1, LOW); digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW); digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  delay(10);

  // Activate and initialize sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  if (!sensor1.begin(0x30)) { // Set new I2C address for sensor 1
    Serial.println("Failed to initialize sensor 1!");
    while (1);
  }
  Serial.println("Sensor 1 initialized at I2C address 0x30");

  // Activate and initialize sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!sensor2.begin(0x31)) { // Set new I2C address for sensor 2
    Serial.println("Failed to initialize sensor 2!");
    while (1);
  }
  Serial.println("Sensor 2 initialized at I2C address 0x31");

  // Activate and initialize sensor 3
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  if (!sensor3.begin(0x32)) { // Set new I2C address for sensor 3
    Serial.println("Failed to initialize sensor 3!");
    while (1);
  }
  Serial.println("Sensor 3 initialized at I2C address 0x32");

  // Activate and initialize sensor 4
  digitalWrite(XSHUT_4, HIGH);
  delay(10);
  if (!sensor4.begin(0x33)) { // Set new I2C address for sensor 4
    Serial.println("Failed to initialize sensor 4!");
    while (1);
  }
  Serial.println("Sensor 4 initialized at I2C address 0x33");

  // Activate and initialize sensor 5
  digitalWrite(XSHUT_5, HIGH);
  delay(10);
  if (!sensor5.begin(0x34)) { // Set new I2C address for sensor 5
    Serial.println("Failed to initialize sensor 5!");
    while (1);
  }
  Serial.println("Sensor 5 initialized at I2C address 0x34");

// Gyroscope
 // Initialize MPU9258 mySensor.beginGyro();
 sensor6.setWire(&Wire);
 sensor6.beginGyro();
 // Calibrate the gyroscope
 calibrateGyro();

  // Print initial success message
  Serial.println("Sensors initialized successfully!");


//Motor 
    pinMode(PWM_m1, OUTPUT);
    pinMode(IN2_m1, OUTPUT);
    pinMode(IN1_m1, OUTPUT);

    pinMode(PWM_m2, OUTPUT);
    pinMode(IN2_m2, OUTPUT);
    pinMode(IN1_m2, OUTPUT);
}


// Gyroscope Code
void calibrateGyro() {
 int numSamples = 1000;
 float sumX =0, sumY =0, sumZ = 0;

 for (int i=0; i < numSamples; i++) 
   {  getGyro_Data(); // Replace with actual gyro data 
      sumX += sensor6.gyroX();
      sumY += sensor6.gyroY();
      sumZ += sensor6.gyroZ();
      delay(1); }

 // Calculate average offset (bias) for each axis
 gyro_x_offset = sumX / numSamples;
 gyro_y_offset = sumY/numSamples;
 gyro_z_offset = sumZ / numSamples;

 Serial.println("Gyroscope Calibration Complete");
 Serial.print("Gyro X Offset: ");
 Serial.println(gyro_x_offset);
 Serial.print("Gyro Y Offset: ");
 Serial.println(gyro_y_offset);
 Serial.print("Gyro Z Offset: ");
 Serial.println(gyro_z_offset);
                        }

void getGyro_Data()  { sensor6.gyroUpdate(); }

 float get_angle_diff(float gyro_z_angle) //cur angle is angle measu
 {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0; // Time step in sec
  previousTime = currentTime;

  // Get the raw gyro and magnetometer data
  getGyro_Data();

  // Correct gyroscope data using static calibration (zero drift correct float gyrož mySensor.gyraz() gyrozoffset: // Z-axis angular veloc
  float gyroZ = sensor6.gyroZ() - gyro_z_offset;

  // Apply complementary filter gyroAnglez gyroAnglez gyrozdt
  gyro_z_angle = gyro_z_angle + gyroZ * dt;

  // Print the results
  Serial.print("Gyro Angle (Z) is ");
  Serial.println(gyro_z_angle);
  delay(100); // Delay for readability

  return gyro_z_angle;
  }


// Motor Code
void turnRight(int spd)
{
    runMotor(0, spd, 0);
    runMotor(1, spd, 1);
}

void turnLeft(int spd)
{
    runMotor(0, spd, 1);
    runMotor(1, spd, 0);
}

void reverse(int spd)
{
    runMotor(0, spd, 1);
    runMotor(1, spd, 1);
}

void forward(int spd)
{
    runMotor(0, spd, 0);
    runMotor(1, spd, 0);
}


void runMotor(int motor, int spd, int dir)
{
  digitalWrite(standby, HIGH);

  boolean dirPin1 = LOW;
  boolean dirPin2 = HIGH;

  if (dir == 1)  // dir=1 backward, dir=0 forward
  {
    dirPin1 = HIGH;
    dirPin2 = LOW;
  }

  if (motor == 1)  // motor=1 m1 right, motor=0 m2 left
  {
    digitalWrite(IN1_m1, dirPin2);
    digitalWrite(IN2_m1, dirPin1);
    analogWrite(PWM_m1, spd);
  }
  else
  {
    digitalWrite(IN1_m2, dirPin2);
    digitalWrite(IN2_m2, dirPin1);
    analogWrite(PWM_m2, spd);
  }
}

void stop()
{
  digitalWrite(IN1_m1, NULL);
  digitalWrite(IN2_m1, NULL);
  analogWrite(PWM_m1, 0);
  digitalWrite(IN1_m2, NULL);
  digitalWrite(IN2_m2, NULL);
  analogWrite(PWM_m2, 0);
}




void loop() {

  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;
  VL53L0X_RangingMeasurementData_t measure4;
  VL53L0X_RangingMeasurementData_t measure5;

  // Take a measurement from sensor 1
  sensor1.rangingTest(&measure1, false);
  if (measure1.RangeStatus != 4) { // RangeStatus 4 means out of range
    Serial.print("Sensor 1: ");
    Serial.print(measure1.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Sensor 1: Out of range");
  }

  // Take a measurement from sensor 2
  sensor2.rangingTest(&measure2, false);
  if (measure2.RangeStatus != 4) {
    Serial.print("Sensor 2: ");
    Serial.print(measure2.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Sensor 2: Out of range");
  }

  // Take a measurement from sensor 3
  sensor3.rangingTest(&measure3, false);
  if (measure3.RangeStatus != 4) {
    Serial.print("Sensor 3: ");
    Serial.print(measure3.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Sensor 3: Out of range");
  }
  // Take a measurement from sensor 4

  sensor4.rangingTest(&measure4, false);
  if (measure4.RangeStatus != 4) {
    Serial.print("Sensor 4: ");
    Serial.print(measure4.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Sensor 4: Out of range");
  }

  // Take a measurement from sensor 5
  sensor5.rangingTest(&measure5, false);
  if (measure5.RangeStatus != 4) {
    Serial.print("Sensor 5: ");
    Serial.print(measure5.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Sensor 5: Out of range");
  }
  // Delay for readability
  delay(500);



// MOTOR CODE

    forward(100); // move forward
    delay(2000);  // for 2 seconds
    stop();       // stop the motors
    delay(250);   // delay between motor runs

  if (gyro_z_angle < 90)//example of turning 90 degrees
   { 
    gyro_z_angle = get_angle_diff(gyro_z_angle) ; 
    turnRight(120); // Turn right
    delay(2000);   // delay between motor runs
    stop();        // stop the motors
    delay(2000);   // delay between motor runs
    } //update difference in angle


    // turnLeft(120); // Turn left
    // delay(2000);   // delay between motor runs
    // stop();        // stop the motors
    // delay(2000);   // delay between motor runs

    // reverse(100); // move forward
    // delay(2000);  // for 2 seconds
    // stop();       // stop the motors
    // delay(250);   // delay between motor runs

            }











