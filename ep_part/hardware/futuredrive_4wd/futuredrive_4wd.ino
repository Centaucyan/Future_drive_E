// Arduino UNO + L293D Motor Shield V1
// M1: rear left, M2: rear right, M3: front right, M4: front left
// Serial commands: W forward, S backward, A left, D right, X stop

const byte LATCH_PIN  = 12;
const byte CLOCK_PIN  = 4;
const byte ENABLE_PIN = 7;
const byte DATA_PIN   = 8;

const byte ULTRASONIC_TRIG_PIN = A0;
const byte ULTRASONIC_ECHO_PIN = A1;
const unsigned long ULTRASONIC_INTERVAL = 200;
unsigned long lastUltrasonicTime = 0;

// Encoder order: M1 rear-left, M2 rear-right, M3 front-right, M4 front-left.
// A2-A5 do not support attachInterrupt() on an Uno, so all four are handled
// by the Port C pin-change interrupt. Only rising edges are counted.
const byte ENCODER_PIN[4] = {A3, A2, A5, A4};
const float ENCODER_PULSES_PER_REV[4] = {
  22.6,  // M1 / A3
  25.2,  // M2 / A2
  21.0,  // M3 / A5
  21.8   // M4 / A4
};

const unsigned long ENCODER_DEBOUNCE_US = 1000;
const unsigned long ENCODER_REPORT_INTERVAL = 200;

volatile unsigned long encoderRawTicks[4] = {0, 0, 0, 0};
volatile long encoderSignedTicks[4] = {0, 0, 0, 0};
volatile unsigned long encoderLastPulseUs[4] = {0, 0, 0, 0};
volatile int8_t encoderDirection[4] = {0, 0, 0, 0};
volatile byte lastEncoderPortState = 0;

long previousEncoderTicks[4] = {0, 0, 0, 0};
unsigned long lastEncoderReportTime = 0;

// Array order: M1, M2, M3, M4
const byte PWM_PIN[4] = {11, 3, 6, 5};
const byte MOTOR_A[4] = {2, 1, 5, 0};
const byte MOTOR_B[4] = {3, 4, 7, 6};

// Direction correction verified on the assembled vehicle.
const int FORWARD_SIGN[4] = {
   1,  // M1
  -1,  // M2
  -1,  // M3
   1   // M4
};

byte motorState = 0;

// PWM range: 0-255
int DRIVE_SPEED = 255;
int TURN_INNER_SPEED = 10;

const unsigned long SAFETY_TIMEOUT = 3000;
unsigned long lastCommandTime = 0;
bool robotMoving = false;


void countEncoderRisingEdge(byte index, unsigned long nowUs)
{
  if (
    nowUs - encoderLastPulseUs[index]
      < ENCODER_DEBOUNCE_US
  ) {
    return;
  }

  encoderLastPulseUs[index] = nowUs;
  encoderRawTicks[index]++;
  encoderSignedTicks[index] += encoderDirection[index];
}


ISR(PCINT1_vect)
{
  byte currentPortState = PINC;
  byte risingEdges =
    (~lastEncoderPortState) & currentPortState;

  lastEncoderPortState = currentPortState;
  unsigned long nowUs = micros();

  // Port C bits: A2=2, A3=3, A4=4, A5=5.
  if (risingEdges & _BV(3)) {
    countEncoderRisingEdge(0, nowUs);  // M1 / A3
  }
  if (risingEdges & _BV(2)) {
    countEncoderRisingEdge(1, nowUs);  // M2 / A2
  }
  if (risingEdges & _BV(5)) {
    countEncoderRisingEdge(2, nowUs);  // M3 / A5
  }
  if (risingEdges & _BV(4)) {
    countEncoderRisingEdge(3, nowUs);  // M4 / A4
  }
}


void reportEncoderData()
{
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastEncoderReportTime;

  if (elapsedMs < ENCODER_REPORT_INTERVAL) {
    return;
  }

  unsigned long rawTicks[4];
  long signedTicks[4];

  noInterrupts();
  for (byte i = 0; i < 4; i++) {
    rawTicks[i] = encoderRawTicks[i];
    signedTicks[i] = encoderSignedTicks[i];
  }
  interrupts();

  Serial.print("ENCODER_TICKS:");
  for (byte i = 0; i < 4; i++) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(rawTicks[i]);
  }
  Serial.println();

  Serial.print("ENCODER_RPM:");
  for (byte i = 0; i < 4; i++) {
    long deltaTicks =
      signedTicks[i] - previousEncoderTicks[i];

    float rpm =
      (deltaTicks * 60000.0) /
      (ENCODER_PULSES_PER_REV[i] * elapsedMs);

    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(rpm, 1);
    previousEncoderTicks[i] = signedTicks[i];
  }
  Serial.println();

  lastEncoderReportTime = now;
}


void sendMotorState()
{
  digitalWrite(LATCH_PIN, LOW);

  for (byte i = 0; i < 8; i++) {
    digitalWrite(CLOCK_PIN, LOW);
    byte bitPosition = 7 - i;
    digitalWrite(DATA_PIN, bitRead(motorState, bitPosition));
    digitalWrite(CLOCK_PIN, HIGH);
  }

  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(LATCH_PIN, HIGH);
}


void setMotorDirection(byte index, int direction)
{
  encoderDirection[index] = direction;

  bitClear(motorState, MOTOR_A[index]);
  bitClear(motorState, MOTOR_B[index]);

  if (direction == 0) {
    return;
  }

  int realDirection = direction * FORWARD_SIGN[index];

  if (realDirection > 0) {
    bitSet(motorState, MOTOR_A[index]);
  } else {
    bitSet(motorState, MOTOR_B[index]);
  }
}


void stopRobot()
{
  for (byte i = 0; i < 4; i++) {
    digitalWrite(PWM_PIN[i], LOW);
    encoderDirection[i] = 0;
  }

  motorState = 0;
  sendMotorState();
  robotMoving = false;

  Serial.println("STOP");
}


void driveRobot(
  int leftDirection,
  int rightDirection,
  int leftSpeed,
  int rightSpeed
)
{
  for (byte i = 0; i < 4; i++) {
    digitalWrite(PWM_PIN[i], LOW);
  }

  motorState = 0;

  // Left wheels: M1 and M4
  setMotorDirection(0, leftDirection);
  setMotorDirection(3, leftDirection);

  // Right wheels: M2 and M3
  setMotorDirection(1, rightDirection);
  setMotorDirection(2, rightDirection);

  sendMotorState();

  Serial.print("LEFT DIR = ");
  Serial.print(leftDirection);
  Serial.print(", RIGHT DIR = ");
  Serial.print(rightDirection);
  Serial.print(", STATE = 0x");
  Serial.println(motorState, HEX);
  Serial.print("LEFT SPEED = ");
  Serial.print(leftSpeed);
  Serial.print(", RIGHT SPEED = ");
  Serial.println(rightSpeed);

  delay(10);

  if (leftDirection != 0) {
    analogWrite(PWM_PIN[0], leftSpeed);   // M1
    analogWrite(PWM_PIN[3], leftSpeed);   // M4
  }

  if (rightDirection != 0) {
    analogWrite(PWM_PIN[1], rightSpeed);  // M2
    analogWrite(PWM_PIN[2], rightSpeed);  // M3
  }

  robotMoving = (leftDirection != 0 || rightDirection != 0);
  lastCommandTime = millis();
}


void moveForward()
{
  driveRobot(1, 1, DRIVE_SPEED, DRIVE_SPEED);
  Serial.println("FORWARD");
}


void moveBackward()
{
  driveRobot(-1, -1, DRIVE_SPEED, DRIVE_SPEED);
  Serial.println("BACKWARD");
}


void turnLeft()
{
  // Both sides move forward; the inside (left) side is slower.
  driveRobot(1, 1, TURN_INNER_SPEED, DRIVE_SPEED);
  Serial.println("LEFT CURVE");
}


void turnRight()
{
  // Both sides move forward; the inside (right) side is slower.
  driveRobot(1, 1, DRIVE_SPEED, TURN_INNER_SPEED);
  Serial.println("RIGHT CURVE");
}


void processSerialCommand(char command)
{
  switch (command) {
    case 'W':
    case 'w':
      moveForward();
      break;

    case 'S':
    case 's':
      moveBackward();
      break;

    case 'A':
    case 'a':
      turnLeft();
      break;

    case 'D':
    case 'd':
      turnRight();
      break;

    case 'X':
    case 'x':
      stopRobot();
      break;
  }
}


void readUltrasonicDistance()
{
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    30000UL
  );

  if (duration == 0) {
    Serial.println("ULTRASONIC_CM:-1");
    return;
  }

  float distanceCm = duration * 0.0343 / 2.0;
  Serial.print("ULTRASONIC_CM:");
  Serial.println(distanceCm, 1);
}


void setup()
{
  Serial.begin(9600);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(DATA_PIN, LOW);

  for (byte i = 0; i < 4; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    digitalWrite(PWM_PIN[i], LOW);

    pinMode(ENCODER_PIN[i], INPUT_PULLUP);
  }

  // Enable pin-change interrupts for A2, A3, A4 and A5 (Port C).
  noInterrupts();
  lastEncoderPortState = PINC;
  PCIFR |= _BV(PCIF1);
  PCMSK1 |=
    _BV(PCINT10) |
    _BV(PCINT11) |
    _BV(PCINT12) |
    _BV(PCINT13);
  PCICR |= _BV(PCIE1);
  interrupts();

  motorState = 0;
  sendMotorState();

  // The shield output-enable pin is active-low.
  digitalWrite(ENABLE_PIN, LOW);

  Serial.println();
  Serial.println("======================");
  Serial.println("4WD SERIAL CONTROL");
  Serial.println("======================");
  Serial.println("W : Forward");
  Serial.println("S : Backward");
  Serial.println("A : Left");
  Serial.println("D : Right");
  Serial.println("X : Stop");
  Serial.println();
  Serial.print("Drive Speed : ");
  Serial.println(DRIVE_SPEED);
  Serial.print("Turn Inner Speed : ");
  Serial.println(TURN_INNER_SPEED);
  Serial.println("======================");
}


void loop()
{
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command != '\n' && command != '\r') {
      processSerialCommand(command);
    }
  }

  if (
    millis() - lastUltrasonicTime >= ULTRASONIC_INTERVAL
  ) {
    lastUltrasonicTime = millis();
    readUltrasonicDistance();
  }

  reportEncoderData();

  if (
    robotMoving &&
    millis() - lastCommandTime >= SAFETY_TIMEOUT
  ) {
    Serial.println("SAFETY TIMEOUT");
    stopRobot();
  }
}
