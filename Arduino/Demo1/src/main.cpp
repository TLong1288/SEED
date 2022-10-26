#define PULSES_PER_REVOLUTION (800.0f)
#define CIRCUMFERENCE (2*M_PI*3)
#define WHEELBASE (14.0f)

#define M1_ENC_A 2 // interrupt pin
#define M1_ENC_B 5
#define M2_ENC_A 3
#define M2_ENC_B 6
#define EN 4

#define DIR2 8
#define DIR1 7

#define positionKp 150
#define positionKi 1
#define positionKd 0

#define rotationKp 100
#define rotationKi 5
#define rotationKd 0

int32_t motorPosition[2] = {0};
int32_t currentPosition = 0;
int32_t currentRotation = 0;
int32_t targetPosition = 0;
int32_t targetRotation = 0;

void m1encoderISR();
void m2encoderISR();

int32_t* matrixMultiply(int32_t* output, const int32_t matrix[2][2], int32_t* input);
void threeVectorAdd(int32_t* output, int32_t* input1, int32_t* input2, int32_t* input3);
int32_t* scalerMultiply(int32_t* output, int32_t* input, int32_t scaler);


int32_t clamp(int32_t value, int32_t maximum, int32_t minimum);
void setMotor(int32_t* motorPower);

int32_t distanceToPulses(float dist);
int32_t degreesToPulses(float deg);

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  // Set the encoder pins to input
  pinMode(M1_ENC_A, INPUT);
  pinMode(M1_ENC_B, INPUT);
  pinMode(M2_ENC_A, INPUT);
  pinMode(M2_ENC_B, INPUT);
  pinMode(EN, OUTPUT);

  // Set the motor pins to output
  // Pins 9 and 10 are the outputs for timer 1
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  digitalWrite(EN, HIGH);

  // Set interrupt
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), m1encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), m2encoderISR, RISING);

  // Configure timer 1
  // 16-bit normal PWM output on channels A and B
  // No prescaler
  TCCR1A = (TCCR1A & ~(0b11110011)) | (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (TCCR1B & ~(0b00011111)) | (1 << CS10) | (1 << WGM13);
  ICR1 = 0xFFFF;
}

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t positionErrorIntegral = 0;
  static int32_t rotationErrorIntegral = 0;
  static uint32_t lastTime = 0;
  static int32_t lastPositionError = 0;
  static int32_t lastRotationError = 0;

  static uint8_t buffer[sizeof(int32_t)*2];
  static int8_t index = -1;

  if(index == -1 && Serial.read() == 'C') index = 0;

  if(Serial.available() > 0 && index != -1){
    uint8_t bytesRead = min(8 - index, Serial.available());
    Serial.readBytes(buffer + index, bytesRead);
    index += bytesRead;

    if(index >= 7){
      int32_t dist = *(int32_t*)(buffer + 4);
      int32_t angle = *(int32_t*)buffer;
      motorPosition[0] = 0;
      motorPosition[1] = 0;
      currentPosition = 0;
      currentRotation = 0;
      positionErrorIntegral = 0;
      rotationErrorIntegral = 0;
      targetPosition = distanceToPulses((float)dist);
      targetRotation = degreesToPulses((float)angle);

      //Serial.println(String(rotationErrorIntegral));

      index = -1;
    }
  }

  int32_t motorPower[2];
  int32_t positionError;
  int32_t rotationError;
  
  // deltaTime is time since the last loop
  // Used for integral and derivitive approximation
  uint32_t deltaTime = millis() - lastTime;
  lastTime = millis();

  positionErrorIntegral += ((targetPosition - currentPosition)*((int32_t)deltaTime));
  positionError = positionKp*(targetPosition - currentPosition) + (positionKi*positionErrorIntegral)/1000 + (positionKd*1000*((targetPosition - currentPosition) - lastPositionError))/deltaTime;
  lastPositionError = (targetPosition - currentPosition);


  rotationErrorIntegral += ((targetRotation - currentRotation)*((int32_t)deltaTime));
  rotationError = rotationKp*(targetRotation - currentRotation) + (rotationKi*rotationErrorIntegral)/1000 + (rotationKd*1000*((targetRotation - currentRotation) - lastRotationError))/deltaTime;
  lastRotationError = targetRotation - currentRotation;
  

  /*if(targetPosition != 0){
    motorPower[0] = positionError;
    motorPower[1] = positionError;
  }else{
    motorPower[0] = rotationError;
    motorPower[1] = -rotationError;
  }*/
  motorPower[0] = positionError + rotationError;
  motorPower[1] = positionError - rotationError;

  //if(printStuff){
  //  Serial.print(String(motorPower[0]) + " " + String(motorPower[1]) + " ");
  //}

  int32_t minimum = min(motorPower[0], motorPower[1]);
  if(minimum < -65535){
    motorPower[0] += -65535 - minimum;
    motorPower[1] += -65535 - minimum;
  }

  int32_t maximum = max(motorPower[0], motorPower[1]);
  if(maximum > 0xFFFF){
    motorPower[0] -= maximum - 65535;
    motorPower[1] -= maximum - 65535;
  }

  /*Serial.print(motorPosition[0]);
  Serial.print("\t");
  Serial.print(motorPosition[1]);
  Serial.print("\t");
  Serial.println(currentRotation);*/

  setMotor(motorPower);
}

void setMotor(int32_t* motorPower){
  // Make sure motor power is within appropriate bounds
  motorPower[0] = clamp(motorPower[0], 65535, -65535);
  motorPower[1] = clamp(motorPower[1], 65535, -65535);

  // Set output pins
  if(motorPower[0] >= 0){
    OCR1A = (uint16_t)(motorPower[0]);
    digitalWrite(DIR1, LOW);
  }else{
    motorPower[0] = -motorPower[0];
    OCR1A = (uint16_t)(motorPower[0]);
    digitalWrite(DIR1, HIGH);
  }

  // Set output pins
  if(motorPower[1] >= 0){
    OCR1B = (uint16_t)(motorPower[1]);
    digitalWrite(DIR2, LOW);
  }else{
    motorPower[1] = -motorPower[1];
    OCR1B = (uint16_t)(motorPower[1]);
    digitalWrite(DIR2, HIGH);
  }
}

int32_t distanceToPulses(float dist){
  return PULSES_PER_REVOLUTION*dist/CIRCUMFERENCE;
}

int32_t degreesToPulses(float deg){
  return distanceToPulses((PI*deg/180)*(WHEELBASE/2))*2;
}

int32_t clamp(int32_t value, int32_t maximum, int32_t minimum){
  if(value > maximum){
    return maximum;
  }else if(value < minimum){
    return minimum;
  }
  return value;
}

void m1encoderISR(){
  if(digitalRead(M1_ENC_B)) {
    motorPosition[0]++;
    //currentRotation++;
  }else{
   motorPosition[0]--;
   //currentRotation--;
  }

  currentPosition = (motorPosition[0] + motorPosition[1])/2;
  currentRotation = motorPosition[0] - motorPosition[1];
}

void m2encoderISR(){
  if(digitalRead(M2_ENC_B)) {
    motorPosition[1]++;
    //currentRotation--;
  }else{
    motorPosition[1]--;
    //currentRotation++;
  }

  currentPosition = (motorPosition[0] + motorPosition[1])/2;
  currentRotation = motorPosition[0] - motorPosition[1];
}
