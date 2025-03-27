#define IR_SENSOR_RIGHT 12
#define IR_SENSOR_LEFT 11
#define BASE_MOTOR_SPEED 100  // Base speed
#define TURN_SPEED 70         // Reduced turning speed for better control
#define CORRECTION_SPEED 50   // Even gentler correction for slight deviations

// TCS3200 sensor pins
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define sensorOut 13

// Variables for color detection
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Speed settings for different colors
#define RED_SPEED 80
#define GREEN_SPEED 140
#define BLUE_SPEED 160

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

// Current base speed that can be modified based on color
int currentBaseSpeed = BASE_MOTOR_SPEED;

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
  
  // Setup for TCS3200 sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
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

int detectColor() {
  // Reading Red Frequency
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  
  // Reading Green Frequency
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Reading Blue Frequency
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  // Print values for debugging
  Serial.print("R: ");
  Serial.print(redFrequency);
  Serial.print(" G: ");
  Serial.print(greenFrequency);
  Serial.print(" B: ");
  Serial.println(blueFrequency);

  // Determine predominant color
  if (redFrequency < greenFrequency && redFrequency < blueFrequency) {
    return 1; // Red color
  } else if (greenFrequency < redFrequency && greenFrequency < blueFrequency) {
    return 2; // Green color
  } else if (blueFrequency < redFrequency && blueFrequency < greenFrequency) {
    return 3; // Blue color
  } else {
    return 0; // Unknown color
  }
}

void loop()
{
  // Update read index
  readIndex = (readIndex + 1) % numReadings;
  
  // Detect color every 500ms to avoid constant checking
  static unsigned long lastColorCheck = 0;
  if (millis() - lastColorCheck > 500) {
    int detectedColor = detectColor();
    
    // Adjust base speed according to detected color
    switch(detectedColor) {
      case 1: // Red
        currentBaseSpeed = RED_SPEED;
        Serial.println("Red detected - Slow speed");
        break;
      case 2: // Green
        currentBaseSpeed = GREEN_SPEED;
        Serial.println("Green detected - Medium speed");
        break;
      case 3: // Blue
        currentBaseSpeed = BLUE_SPEED;
        Serial.println("Blue detected - Fast speed");
        break;
      default: // Default speed if color not recognized
        currentBaseSpeed = BASE_MOTOR_SPEED;
        Serial.println("No specific color detected - Default speed");
    }
    
    lastColorCheck = millis();
  }
  
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
  
  // Calculate motor speeds with PID correction using the current base speed
  int rightMotorSpeed = currentBaseSpeed - correction * 30;
  int leftMotorSpeed = currentBaseSpeed + correction * 30;
  
  // Limit motor speeds
  rightMotorSpeed = constrain(rightMotorSpeed, -currentBaseSpeed, currentBaseSpeed);
  leftMotorSpeed = constrain(leftMotorSpeed, -currentBaseSpeed, currentBaseSpeed);
  
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