/*Multiple DC motors via serial communication*/

const int pwmPin = 2;
const int in1Pin = 22;
const int in2Pin = 23;

unsigned long lastCommandTime = 0;
const unsigned long timeout = 500; // ms

String input = "";

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

  pinMode(3, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);

  Serial.println("READY");   // handshake
}

void setMotor(int motor, int speed) {

  speed = constrain(speed, -100, 100);

  int pwm = map(abs(speed), 0, 100, 0, 255);
  bool dir = speed >= 0;

  if (motor == 1) {
    digitalWrite(22, dir);
    digitalWrite(23, !dir);
    analogWrite(2, pwm);
  }

  else if (motor == 2) {
    digitalWrite(24, dir);
    digitalWrite(25, !dir);
    analogWrite(3, pwm);
  }
}

void parseCommand(String cmd) {
  lastCommandTime = millis();
  // Expect: D,50,-30

  if (!cmd.startsWith("D,")) return;

  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);

  int m1 = cmd.substring(firstComma + 1, secondComma).toInt();
  int m2 = cmd.substring(secondComma + 1).toInt();

  setMotor(1, m1);
  setMotor(2, m2);
}

void loop() {
  /*if (millis() - lastCommandTime > timeout) {
  setMotor(1, 0);
  setMotor(2, 0);
}*/
  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {
      parseCommand(input);
      input = "";
    }
    else {
      input += c;
    }
  }
}
