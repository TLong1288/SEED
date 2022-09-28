#include <Arduino.h>

#define PULSES_PER_REVOLUTION 800

#define ENC_A 2 // interrupt pin
#define ENC_B 5
#define EN 4

#define DIR 8

#define Kp 85
#define Ki 10
#define Kd 8UL

int32_t targetPosition = 800;
int32_t position = 0;

// ISR that runs whenever a pin change occurs on ENC_A pin
// Increments/decrements position
void encoderISR();

// Sets the motor to speed specified by motorPower
// motorPower should be a signed int between 0xFFFF and -0xFFFF
void setMotor(int32_t motorPower);

// Clamps value between the max and min
int32_t clamp(int32_t value, int32_t maximum, int32_t minimum);

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  // Set the encoder pins to input
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(EN, OUTPUT);

  // Set the motor pins to output
  // Pins 9 and 10 are the outputs for timer 1
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(EN, HIGH);

  // Set interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  // Configure timer 1
  // 16-bit normal PWM output on channels A and B
  // No prescaler
  TCCR1A = (TCCR1A & ~(0b11110000)) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (TCCR1B & ~(0b00011111)) | (1 << CS10);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t errorIntegral = 0;
  static uint32_t lastTime = 0;
  static int32_t lastPositionError = 0;
  int32_t motorPower;

  // deltaTime is time since the last loop
  // Used for integral and derivitive approximation
  uint32_t deltaTime = millis() - lastTime;
  lastTime = millis();

  // Integrate the position error
  errorIntegral += ((targetPosition - position)*((int32_t)deltaTime));

  // Calculate the motor power from pid constants
  motorPower = Kp*(targetPosition - position) + (Ki*errorIntegral)/1000 + (Kd*8000UL*((targetPosition - position) - (lastPositionError)))/((int32_t)deltaTime);
  
  // Set the previous position error for the D term
  lastPositionError = targetPosition - position;

  // Update the motor
  setMotor(motorPower);
}

void setMotor(int32_t motorPower){
  // Make sure motor power is within appropriate bounds
  motorPower = clamp(motorPower, 65535, -65535);

  // Set output pins
  if(motorPower >= 0){
    OCR1B = (uint16_t)motorPower;
    digitalWrite(DIR, LOW);
  }else{
    motorPower = -motorPower;
    OCR1B = (uint16_t)motorPower;
    digitalWrite(DIR, HIGH);
  }
}

int32_t clamp(int32_t value, int32_t maximum, int32_t minimum){
  if(value > maximum){
    return maximum;
  }else if(value < minimum){
    return minimum;
  }
  return value;
}

void encoderISR(){
  if(digitalRead(ENC_B)) position++;
  else position--;
}