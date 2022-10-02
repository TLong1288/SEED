#include <Arduino.h>
//#include <Encoder.h>

#define PULSES_PER_REVOLUTION 800

#define ENC_A 2 // interrupt pin
#define ENC_B 5
#define EN 4

#define DIR 8

#define DEADBAND 10

#define Kp 850
#define Ki 10
#define Kd 8UL

//////////////GLOBALS////////////////
int32_t targetPosition = 0;
int32_t position = 0;
int32_t num = 0;

bool DataRead;
byte data;

//////////////HEADERS////////////////
// ISR that runs whenever a pin change occurs on ENC_A pin
// Increments/decrements position
void encoderISR();

// Sets the motor to speed specified by motorPower
// motorPower should be a signed int between 0xFFFF and -0xFFFF
void setMotor(int32_t motorPower);

// Clamps value between the max and min
int32_t clamp(int32_t value, int32_t maximum, int32_t minimum);
//////////////END HEADERS////////////////

//////////////FUNCTIONS////////////////
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
  TCCR1A = (TCCR1A & ~(0b11110011)) | (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (TCCR1B & ~(0b00011111)) | (1 << CS10) | (1 << WGM13);
  ICR1 = 0xFFFF;
}


/////////////////////////////////////////////////
////////////////////MAIN/////////////////////////
/////////////////////////////////////////////////
void loop() {

  // COMM
  if(DataRead){
    if(data) targetPosition = (int32_t)(data - 1)*PULSES_PER_REVOLUTION/4;
    Serial.println(position);
    DataRead = false;
  }
  
  // put your main code here, to run repeatedly:
  static int32_t errorIntegral = 0;
  static uint32_t lastTime = 0;
  static int32_t lastPositionError = 0;
  int32_t motorPower;
  static uint32_t lastSentTime = 0;

  // Send thecurrent position of the motor every 100ms
  /*if(millis() - lastSentTime > 100){
    Serial.print(String(position));
    lastSentTime = millis();
  }*/

  // deltaTime is time since the last loop
  // Used for integral and derivitive approximation
  uint32_t deltaTime = millis() - lastTime;
  lastTime = millis();

  // Integrate the position error
  errorIntegral += ((targetPosition - position)*((int32_t)deltaTime));

  // Calculate the motor power from pid constants
  motorPower = Kp*(targetPosition - position) + (Ki*errorIntegral)/1000 + (Kd*8000UL*((targetPosition - position) - (lastPositionError)))/((int32_t)deltaTime);
  
  if(abs(targetPosition - position) < DEADBAND){
    motorPower = 0;
  }

  // Set the previous position error for the D term
  lastPositionError = targetPosition - position;

  // Update the motor
  setMotor(motorPower);
} //END LOOP


// COMM
void serialEvent(){
  if(Serial.available() > 0){
    data = Serial.read();
    DataRead = true;
  }
  Serial.flush();
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

//////////////END FUNCTIONS////////////////
