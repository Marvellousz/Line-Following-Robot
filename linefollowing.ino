#define IR_SENSOR_RIGHT 12
#define IR_SENSOR_LEFT 11
#define BASE_MOTOR_SPEED 100  // Base speed
#define TURN_SPEED 70         // Reduced turning speed for better control
#define CORRECTION_SPEED 50   // Even gentler correction for slight deviations

// PID control variables
float Kp = 0.5;               // Proportional constant
float Ki = 0.1;               // Integral constant
float Kd = 0.2;               // Derivative constant
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

//Right motor
int enableRightMotor = 6;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

//Left motor
int enableLeftMotor = 5;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

// For smoothing sensor readings
const int numReadings = 3;
int rightReadings[numReadings];
int leftReadings[numReadings];
int readIndex = 0;

void setup()
{
  Serial.begin(9600);  // Add serial for debugging
  
  // Set PWM frequency for better motor control at low speeds
  TCCR0B = TCCR0B & B11111000 | B00000010;
  
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  
  // Initialize the readings arrays
  for (int i = 0; i < numReadings; i++) {
    rightReadings[i] = 0;
    leftReadings[i] = 0;
  }
  
  rotateMotor(0, 0);
  
  // Small delay before starting
  delay(1000);
}

// Function to get smoothed sensor readings
bool readSensor(int sensorPin, int readings[]) {
  readings[readIndex] = digitalRead(sensorPin);
  
  // Calculate the sum
  int sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += readings[i];
  }
  
  // Return true if majority of readings are HIGH
  return (sum > numReadings/2);
}

void loop()
{
  // Update read index
  readIndex = (readIndex + 1) % numReadings;
  
  // Get smoothed sensor readings
  bool rightSensorOnLine = readSensor(IR_SENSOR_RIGHT, rightReadings);
  bool leftSensorOnLine = readSensor(IR_SENSOR_LEFT, leftReadings);
  
  // Calculate error: -1 for line on left, 0 for centered, 1 for line on right
  if (!rightSensorOnLine && !leftSensorOnLine) {
    // Both sensors off the line, maintain previous error for continuity
    // This helps the robot continue in the same direction when temporarily losing the line
  } 
  else if (!rightSensorOnLine && leftSensorOnLine) {
    error = -1;  // Line is to the left
  } 
  else if (rightSensorOnLine && !leftSensorOnLine) {
    error = 1;   // Line is to the right
  } 
  else {
    error = 0;   // Line is centered
  }
  
  // PID calculation
  integral = integral + error;
  derivative = error - previousError;
  
  // Limit integral to prevent windup
  if (integral > 10) integral = 10;
  if (integral < -10) integral = -10;
  
  // Calculate correction using PID
  float correction = Kp * error + Ki * integral + Kd * derivative;
  
  // Calculate motor speeds with PID correction
  int rightMotorSpeed = BASE_MOTOR_SPEED - correction * 30;
  int leftMotorSpeed = BASE_MOTOR_SPEED + correction * 30;
  
  // Limit motor speeds
  rightMotorSpeed = constrain(rightMotorSpeed, -BASE_MOTOR_SPEED, BASE_MOTOR_SPEED);
  leftMotorSpeed = constrain(leftMotorSpeed, -BASE_MOTOR_SPEED, BASE_MOTOR_SPEED);
  
  // Apply motor speeds
  rotateMotor(rightMotorSpeed, leftMotorSpeed);
  
  // Save current error for next iteration
  previousError = error;
  
  // Small delay to prevent erratic behavior
  delay(10);
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }
  
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}