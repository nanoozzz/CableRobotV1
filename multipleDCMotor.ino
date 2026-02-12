/*-------------------------------------------
  Multiple DC motors with fast velocity PID
  FULL QUADRATURE VERSION (x4 decoding)
--------------------------------------------*/

const int pwmPin[4] = {2, 5, 4, 3};
const int in1Pin[4] = {47, 35, 39, 43};
const int in2Pin[4] = {49, 37, 41, 45};

// Encoder pins
const int encA[4] = {27, 31, 33, 29};
const int encB[4] = {26, 30, 32, 28};

// Encoder variables
volatile long encoderCount[4] = {0,0,0,0};
volatile uint8_t lastState[4] = {0,0,0,0};

// Quadrature lookup table (FAST + reliable)
const int8_t quadTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

float targetVelocity[4] = {0,0,0,0};
float measuredVelocity[4] = {0,0,0,0};

// PID gains
/*
float Kp = 3;
float Ki = 0.8;
float Kd = 0.01;
float Kv_pos = 0.12;
float Kv_neg = 0.16; 
*/
float Kp[4] = {0.45,0.5,0.5,0.5};
float Ki[4] = {0.1,0.1,0.1,0.1};
float Kd[4] = {0.015,0.01,0.01,0.01};
float Kv_pos[4] = {0.014,0.014,0.014,0.014};
float Kv_neg[4] = {0.018,0.018,0.018,0.018};

float Fc_pos[4] = {0,0,0,0};   // PWM needed to keep moving +
float Fc_neg[4] = {0,0,0,0};   // usually larger

float dFilt[4] = {0,0,0,0};
const float D_ALPHA = 0.3;   // 0.2–0.3 is excellent

float prevError[4] = {0,0,0,0};
float integral[4] = {0,0,0,0};

unsigned long lastTime = 0;
const unsigned long LOOP_INTERVAL = 2; // 500Hz

unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT = 300;

const int PWM_MAX = 255;
const int BREAKAWAY = 102;
const float I_MAX = 1000; 

String input = "";

// IMPORTANT: datasheet already assumes x4 decoding
const int COUNTS_PER_REV = 48 * 75;

/*-------------------------------------------
   QUADRATURE DECODER
--------------------------------------------*/

inline void updateEncoder(uint8_t i){

  uint8_t A = digitalRead(encA[i]);
  uint8_t B = digitalRead(encB[i]);

  uint8_t state = (A << 1) | B;
  uint8_t index = (lastState[i] << 2) | state;

  encoderCount[i] += quadTable[index];
  lastState[i] = state;
}

void encoderISR0(){ updateEncoder(0); }
void encoderISR1(){ updateEncoder(1); }
void encoderISR2(){ updateEncoder(2); }
void encoderISR3(){ updateEncoder(3); }

/*-------------------------------------------
  Setup
--------------------------------------------*/
void setup() {

  Serial.begin(115200);

  for(int i=0; i<4; i++){
    pinMode(pwmPin[i], OUTPUT);
    pinMode(in1Pin[i], OUTPUT);
    pinMode(in2Pin[i], OUTPUT);

    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }

  // Initialize encoder states BEFORE interrupts
  for(int i=0;i<4;i++){
    uint8_t A = digitalRead(encA[i]);
    uint8_t B = digitalRead(encB[i]);
    lastState[i] = (A<<1) | B;
  }

  // Attach BOTH channels → both edges
  attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[0]), encoderISR0, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[1]), encoderISR1, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[2]), encoderISR2, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encA[3]), encoderISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[3]), encoderISR3, CHANGE);

  lastTime = millis();

  Serial.println("READY");
}

/*-------------------------------------------
  Set motor PWM + direction
--------------------------------------------*/
void setMotorRaw(int motor, float pwm){

  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  bool dir = pwm >= 0;

  analogWrite(pwmPin[motor], abs(pwm));
  digitalWrite(in1Pin[motor], dir);
  digitalWrite(in2Pin[motor], !dir);
}

/*-------------------------------------------
  Parse command from Python
--------------------------------------------*/
void parseCommand(String cmd) {

  lastCommandTime = millis();

  if (!cmd.startsWith("D,")) return;

  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);
  int comma3 = cmd.indexOf(',', comma2 + 1);
  int comma4 = cmd.indexOf(',', comma3 + 1);

  targetVelocity[0] = cmd.substring(comma1 + 1, comma2).toFloat();
  targetVelocity[1] = cmd.substring(comma2 + 1, comma3).toFloat();
  targetVelocity[2] = cmd.substring(comma3 + 1, comma4).toFloat();
  targetVelocity[3] = cmd.substring(comma4 + 1).toFloat();
}

/*-------------------------------------------
  FAST PID LOOP
--------------------------------------------*/
void updatePID(){

  unsigned long now = millis();
  static long prevCount[4] = {0,0,0,0};

  float dt = (now - lastTime)/1000.0;
  if(dt <= 0) return;

  for(int i=0;i<4;i++){

    long count = encoderCount[i];

    measuredVelocity[i] = (count - prevCount[i]) / dt;
    prevCount[i] = count;

    float err = targetVelocity[i] - measuredVelocity[i];

    // Filtered derivative
    float rawD = (err - prevError[i]) / dt;
    dFilt[i] = D_ALPHA * rawD + (1 - D_ALPHA) * dFilt[i];

    //---------------------------------------
    // Feedforward
    //---------------------------------------

    float ff = (targetVelocity[i] >= 0)
           ? Kv_pos[i] * targetVelocity[i]
           : Kv_neg[i] * targetVelocity[i];

    //---------------------------------------
    // ⭐ Coulomb friction compensation
    //---------------------------------------

    float friction = 0;

    if(targetVelocity[i] > 0)
        friction = Fc_pos[i];
    else if(targetVelocity[i] < 0)
        friction = -Fc_neg[i];

    //---------------------------------------
    // Integral with clamp
    //---------------------------------------

    float newIntegral = integral[i] + err * dt;
    newIntegral = constrain(newIntegral, -I_MAX, I_MAX);

    //---------------------------------------
    // Control output
    //---------------------------------------

    float u = ff
            + friction
            + Kp[i]*err
            + Ki[i]*newIntegral
            + Kd[i]*dFilt[i];

    float u_clamped = constrain(u, -PWM_MAX, PWM_MAX);

    //---------------------------------------
    // Anti-windup (clean version)
    //---------------------------------------

    if(u == u_clamped)   // only integrate if NOT saturated
        integral[i] = newIntegral;

    //---------------------------------------
    // Zero command cleanup
    //---------------------------------------

    if(targetVelocity[i] == 0){
        u_clamped = 0;
        integral[i] = 0;
    }

    prevError[i] = err;

    setMotorRaw(i, u_clamped);
  }

  lastTime = now;
}


/*-------------------------------------------
  Main loop
--------------------------------------------*/
void loop() {

  // watchdog
  if (millis() - lastCommandTime > TIMEOUT) {
    for(int i=0;i<4;i++) setMotorRaw(i,0);
  }

  // serial read
  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      parseCommand(input);
      input = "";
    }
    else input += c;
  }

  // fast loop
  static unsigned long lastLoop = 0;

  if(millis() - lastLoop >= LOOP_INTERVAL){
    updatePID();
    lastLoop = millis();
  }
}
