/*-------------------------------------------
  Multiple DC motors with fast velocity PID
  X1 ENCODING (Encoder A rising only)
--------------------------------------------*/
// ================= 10 SECOND LOGGER =================

#define LOG_DURATION_US 10000000UL   // 10 seconds
#define LOOP_HZ 500
#define LOG_SIZE (LOG_DURATION_US / 5000)   // 5000 samples

unsigned long logTime[LOG_SIZE];
float logTarget[LOG_SIZE];
float logMeasured[LOG_SIZE];

volatile int logIndex = 0;

bool experimentRunning = false;
bool experimentFinished = false;

unsigned long experimentStartTime = 0;

const int pwmPin[4] = {2, 5, 4, 3};
const int in1Pin[4] = {47, 35, 39, 43};
const int in2Pin[4] = {49, 37, 41, 45};

// Encoder pins
const int encA[4] = {27, 33, 31, 29};
const int encB[4] = {26, 32, 30, 28};

// Encoder variables
volatile long encoderCount[4] = {0,0,0,0};

// ================= CONTROL VARIABLES =================

float targetVelocity[4] = {0,0,0,0};
float measuredVelocity[4] = {0,0,0,0};
float measuredVelocityFilt[4] = {0,0,0,0};
float measuredVelocityPrev[4] = {0,0,0,0};

// PID gains, test 16-8-0.01
float Kpp[4]      = {16, 16, 18, 16};
float Kip[4]      = {10, 12.0, 12.0, 12.0};
float Kdp[4]      = {0.04, 0.08, 0.06, 0.06};

float Kpn[4]      = {16, 16.0, 18.0, 16.0};
float Kin[4]      = {8, 10.0, 10.0, 10.0};
float Kdn[4]      = {0.04, 0.08, 0.06, 0.06};

float Kv_pos[4]  = {0.00, 0.0, 0.0, 0.0};
float Kv_neg[4]  = {0, 0.0, 0.0, 0.0};

float Fc_pos[4]  = {0,0,0,0};
float Fc_neg[4]  = {0,0,0,0};

float dFilt[4]   = {0,0,0,0};

float prevError[4] = {0,0,0,0};
float integral[4]  = {0,0,0,0};

unsigned long lastTime = 0;
const unsigned long LOOP_INTERVAL = 2000; // 500 Hz

unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT = 200000; // microsec

const int PWM_MAX = 255;
const float I_MAX = 1000;

// IMPORTANT: Now using x1 decoding (divide previous CPR by 4)
const float COUNTS_PER_SEC_TO_RPM = 1/(48/4 * 74.83) * 60;

String input = "";

/*-------------------------------------------
   ENCODER ISR (X1 Decoding)
--------------------------------------------*/

inline void updateEncoder(uint8_t i){

  // Called only on A rising edge
  if (digitalRead(encA[i]) > 0)
      encoderCount[i]++;     // forward
  else
      encoderCount[i]--;     // reverse
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

  // Attach interrupt ONLY to Encoder A (RISING)
  attachInterrupt(digitalPinToInterrupt(encB[0]), encoderISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(encB[1]), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encB[2]), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(encB[3]), encoderISR3, RISING);

  lastTime = micros();
  experimentStartTime = micros();
  experimentRunning = true;
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

  lastCommandTime = micros();

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

  unsigned long now = micros();
  static long prevCount[4] = {0,0,0,0};

  //float dt = ((float)(now - lastTime))/1.0e6;
  const float dt = 0.002;
  if(dt <= 0) return;

  for(int i=0;i<4;i++){
    long count = 0;
    noInterrupts();
    count = encoderCount[i];
    interrupts();

    measuredVelocity[i] = (float)((count - prevCount[i]) / dt); 
    measuredVelocity[i] = measuredVelocity[i]/897.96*60.0;// in RPM
    prevCount[i] = count;
    measuredVelocityFilt[i] = 0.882*measuredVelocityFilt[i] + 0.0591*measuredVelocity[i] + 0.0591*measuredVelocityPrev[i]; // 20Hz low pass
    measuredVelocityPrev[i] = measuredVelocity[i];

    float err = targetVelocity[i] - measuredVelocityFilt[i];

    // Derivative (filtered)
    float rawD = (err - prevError[i]) / dt;
    dFilt[i] = rawD;

    // Feedforward
    float ff = (targetVelocity[i] >= 0)
             ? Kv_pos[i] * targetVelocity[i]
             : -Kv_neg[i] * targetVelocity[i];

    // Friction compensation
    float friction = 0;
    if(targetVelocity[i] > 0)
        friction = Fc_pos[i];
    else if(targetVelocity[i] < 0)
        friction = -Fc_neg[i];

    // Integral (anti-windup)
    float newIntegral = integral[i] + err * dt;
    newIntegral = constrain(newIntegral, -I_MAX, I_MAX);

    // Control law
    float u = 0;
    if (targetVelocity[i] >= 0) {
      u = ff
          + friction
          + Kpp[i]*err
          + Kip[i]*newIntegral
          + Kdp[i]*dFilt[i];
    }
    else {
      u = ff
          + friction
          + Kpn[i]*err
          + Kin[i]*newIntegral
          + Kdn[i]*dFilt[i];
    }

    float u_clamped = constrain(u, -PWM_MAX, PWM_MAX);

    if(u == u_clamped)
        integral[i] = newIntegral;

    // Zero command handling
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

  // Watchdog
  //if (micros() - lastCommandTime > TIMEOUT) {
  //  for(int i=0;i<4;i++) setMotorRaw(i,0);
  //}
  
  // Serial input
  
  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      parseCommand(input);
      input = "";
    }
    else input += c;
  }

  // 500 Hz control loop
  static unsigned long lastLoop = 0;

  //if(experimentRunning && micros() - lastLoop >= LOOP_INTERVAL){
  if(micros() - lastLoop >= LOOP_INTERVAL){
    // watchdog
    if (micros() - lastCommandTime > TIMEOUT) {

        for(int i=0;i<4;i++){
            targetVelocity[i] = 0;
            integral[i] = 0;
        }
    }
    updatePID();
    lastLoop += LOOP_INTERVAL;   // precise timing

    unsigned long now = micros();

    // ===== LOG DATA =====
    /*if(logIndex < LOG_SIZE){
        logTime[logIndex] = now - experimentStartTime;
        logTarget[logIndex] = targetVelocity[1];
        logMeasured[logIndex] = measuredVelocityFilt[1];
        logIndex++;
    }

    // ===== STOP AFTER 10 SECONDS =====
    if(now - experimentStartTime >= LOG_DURATION_US){

        experimentRunning = false;
        experimentFinished = true;

        // Stop all motors
        for(int i=0;i<4;i++){
            setMotorRaw(i, 0);
        }
    }
  }


    if(experimentFinished){

    Serial.println("time_us,target,measured");

    for(int i=0; i<logIndex; i++){
        Serial.print(logTime[i]);
        Serial.print(",");
        Serial.print(logTarget[i]);
        Serial.print(",");
        Serial.println(logMeasured[i]);
    }

    experimentFinished = false;*/   // prevent re-printing
  }
}
