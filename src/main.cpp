#include <Arduino.h>

// if using teensy
#ifdef TEENSYDUINO
#include <util/atomic.h>
#endif // TEENSYDUINO

enum Motor { M1, M2, M3, M4 };

/* IDLE: no control, all motors off
 * SPEED: control speed
 * POSITION: control position
 */
enum operatingMode { IDLE, SPEED, POSITION, CALIBRATE };
operatingMode mode = IDLE;

// last command time
long lastCommandTime = 0;

// motor 1 - Front Left
// motor 2 - Front Right
// motor 3 - Back Left
// motor 4 - Back Right
#if defined(TEENSYDUINO)
int enc_a_pins[] = {5, 3, 1, 7};  // YELLOW
int enc_b_pins[] = {4, 11, 0, 6}; // WHITE
int pwm_pins[] = {22, 10, 14, 19};
int in1_pins[] = {21, 8, 15, 18};
int in2_pins[] = {20, 9, 16, 17};
#elif defined(ARDUINO_ARCH_MBED)
int enc_a_pins[] = {2, 4, 6, 8}; // YELLOW
int enc_b_pins[] = {3, 5, 7, 9}; // WHITE
int pwm_pins[] = {15, 21, 16, 10};
int in1_pins[] = {14, 19, 18, 11};
int in2_pins[] = {13, 20, 17, 12};
#else
#error "This board has no defined pins."
#endif

int min_power_for_movement[] = {0, 0, 0, 0};

float kp[] = {1, 1, 1, 1};
float kd[] = {0.025, 0.025, 0.025, 0.025};
float ki[] = {0.0, 0.0, 0.0, 0.0};
int motor_speed[] = {0, 0, 0, 0};
int motor_dir[] = {0, 0, 0, 0};

int steps = 0;
int num_steps_per_print = 1000;

int target[] = {0, 0, 0, 0};
int pos[] = {0, 0, 0, 0};
volatile int posi[] = {0, 0, 0, 0}; // specify posi as volatile:
long prevT[] = {0, 0, 0, 0};
float eprev[] = {0, 0, 0, 0};
float eintegral[] = {0, 0, 0, 0};

void setMotor(int dir, int pwmVal, int i);
void readEncoder1();
void readEncoder2();
void readEncoder3();
void readEncoder4();

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 4; i++) {
    pinMode(enc_a_pins[i], INPUT);
    pinMode(enc_b_pins[i], INPUT);
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(in1_pins[i], OUTPUT);
    pinMode(in2_pins[i], OUTPUT);
  }

  Serial.println("Pin Setup complete");

  attachInterrupt(digitalPinToInterrupt(enc_a_pins[0]), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_a_pins[1]), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_a_pins[2]), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(enc_a_pins[3]), readEncoder4, RISING);

  Serial.println("Interrupts attached");
}

void loop() {
  bool print = steps % num_steps_per_print == 0;
  if (mode != CALIBRATE) {
    // read serial commands (if any)
    if (Serial.available()) {
      lastCommandTime = millis();
      char c = Serial.read();
      if (c == 't') { // set target
        target[0] = Serial.parseInt();
        target[1] = Serial.parseInt();
        target[2] = Serial.parseInt();
        target[3] = Serial.parseInt();
        Serial.print("OK - New Targets: ");
        Serial.print(target[0]);
        Serial.print(" ");
        Serial.print(target[1]);
        Serial.print(" ");
        Serial.print(target[2]);
        Serial.print(" ");
        Serial.println(target[3]);
        mode = POSITION;
      } else if (c == 'r') { // reset encoders
        posi[0] = 0;
        posi[1] = 0;
        posi[2] = 0;
        posi[3] = 0;
        Serial.println("OK - Encoders reset");
        mode = IDLE;
      } else if (c == 'a') { // add to target
        target[0] += Serial.parseInt();
        target[1] += Serial.parseInt();
        target[2] += Serial.parseInt();
        target[3] += Serial.parseInt();
        Serial.print("OK - New Targets: ");
        Serial.print(target[0]);
        Serial.print(" ");
        Serial.print(target[1]);
        Serial.print(" ");
        Serial.print(target[2]);
        Serial.print(" ");
        Serial.println(target[3]);
        mode = POSITION;
      } else if (c == 's') { // set speed
        motor_speed[0] = Serial.parseInt();
        motor_speed[1] = Serial.parseInt();
        motor_speed[2] = Serial.parseInt();
        motor_speed[3] = Serial.parseInt();
        motor_dir[0] = motor_speed[0] < 0 ? -1 : 1;
        motor_dir[1] = motor_speed[1] < 0 ? -1 : 1;
        motor_dir[2] = motor_speed[2] < 0 ? -1 : 1;
        motor_dir[3] = motor_speed[3] < 0 ? -1 : 1;
        Serial.print("OK - New Speeds: ");
        Serial.print(motor_speed[0]);
        Serial.print(" ");
        Serial.print(motor_dir[0]);
        Serial.print(" ");
        Serial.print(motor_speed[1]);
        Serial.print(" ");
        Serial.print(motor_dir[1]);
        Serial.print(" ");
        Serial.print(motor_speed[2]);
        Serial.print(" ");
        Serial.print(motor_dir[2]);
        Serial.print(" ");
        Serial.print(motor_speed[3]);
        Serial.print(" ");
        Serial.println(motor_dir[3]);
        mode = SPEED;
      } else if (c == 'p') { // set PID P value
        int motor = Serial.parseInt();
        float p = Serial.parseFloat();
        kp[motor] = p;
        Serial.print("OK - Motor: ");
        Serial.print(motor);
        Serial.print(" New kp: ");
        Serial.println(p);
      } else if (c == 'd') { // set PID D value
        int motor = Serial.parseInt();
        float d = Serial.parseFloat();
        kd[motor] = d;
        Serial.print("OK - Motor: ");
        Serial.print(motor);
        Serial.print(" New kd: ");
        Serial.println(d);
      } else if (c == 'i') { // set PID I value
        int motor = Serial.parseInt();
        float i = Serial.parseFloat();
        ki[motor] = i;
        Serial.print("OK - Motor: ");
        Serial.print(motor);
        Serial.print(" New ki: ");
        Serial.println(i);
      } else if (c == 'c') { // run calibration
        mode = CALIBRATE;
        Serial.println("OK - Calibrating motors");
      } else { // invalid command
        Serial.print("Invalid command received: '");
        Serial.print(c);
        Serial.println("'");
      }
    } else if (lastCommandTime > 0 && millis() - lastCommandTime > 1000) {
      mode = IDLE;
      lastCommandTime = 0;
    }
  }

  if (print) {
    Serial.print("Step: ");
    Serial.print(steps);
  }

  for (int i = 0; i < 4; i++) {
#if defined(TEENSYDUINO)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pos[i] = posi[i]; }
// if using rp2040
#elif defined(ARDUINO_ARCH_MBED)
    // TODO: implement atomic block for RP2040
    pos[i] = posi[i];
#else
#error "This board is not supported."
#endif

    if (print) {
      Serial.print(" Motor ");
      Serial.print(i);
      Serial.print(" pos:");
      Serial.print(pos[i]);
    }

    switch (mode) {
    case IDLE: {
      // disable motor
      motor_speed[i] = 0;
      motor_dir[i] = 0;
      if (print) {
        Serial.print(" IDLE");
      }
      break;
    }

    case SPEED: {
      // set motor speed
      // motor speed is already set in the serial command
      break;
    }

    case POSITION: {
      // time difference
      long currT = micros();
      float deltaT = ((float)(currT - prevT[i])) / (1.0e6);
      prevT[i] = currT;

      // error
      int e = pos[i] - target[i];

      // derivative
      float dedt = (e - eprev[i]) / (deltaT);

      // integral
      eintegral[i] = eintegral[i] + e * deltaT;

      // control signal
      float u = kp[i] * e + kd[i] * dedt + ki[i] * eintegral[i];

      // store previous error
      eprev[i] = e;

      // motor power
      int pwr = fabs(u);
      if (pwr > 255) {
        pwr = 255;
      } else if (pwr < 0) {
        pwr = 0;
      } else if (pwr < 100) {
        pwr = 0;
      }
      motor_speed[i] = (int)pwr;

      // motor direction
      if (u < 0) {
        motor_dir[i] = 1;
      } else if (u > 0) {
        motor_dir[i] = -1;
      }

      if (print) {
        Serial.print(" target:");
        Serial.print(target[i]);
      }
      break;
    }

    case CALIBRATE: {
      // calibrate motor
      int starting_pos = pos[i];
      int speed = 0;
      int dir = 1;
      while (true) {
        setMotor(dir, speed, i);
        delay(100);
        if (posi[i] > starting_pos + 10) {
          min_power_for_movement[i] = speed;
          if (i == 3) {
            mode = IDLE;
            Serial.print("Calibration complete ");
            Serial.print("min_power_for_movement: ");
            Serial.print(min_power_for_movement[0]);
            Serial.print(" ");
            Serial.print(min_power_for_movement[1]);
            Serial.print(" ");
            Serial.print(min_power_for_movement[2]);
            Serial.print(" ");
            Serial.println(min_power_for_movement[3]);
          }
          break;
        }
        speed += 10;
        if (speed > 255) {
          mode = IDLE;
          Serial.print("Calibration failed, motor ");
          Serial.print(i);
          Serial.println(" did not move.");
        }
      }
      break;
    }
    default:
      break;
    }

    // signal the motor
    setMotor(motor_dir[i], motor_speed[i], i);

    if (print) {
      Serial.print(" speed:");
      Serial.print(motor_speed[i]);
      Serial.print(" dir:");
      Serial.print(motor_dir[i]);
      Serial.print("    ");
    }
  }
  
  if (print) {
    Serial.println();
  }
  steps++;
}

void setMotor(int dir, int pwmVal, int i) {
  if (pwmVal < min_power_for_movement[i]) {
    pwmVal = 0;
    dir = 0;
  }
  analogWrite(pwm_pins[i], abs(pwmVal));
  if (dir > 0) {
    digitalWrite(in2_pins[i], LOW);
    digitalWrite(in1_pins[i], HIGH);
  } else if (dir < 0) {
    digitalWrite(in1_pins[i], LOW);
    digitalWrite(in2_pins[i], HIGH);
  } else {
    digitalWrite(in1_pins[i], LOW);
    digitalWrite(in2_pins[i], LOW);
  }
}

void readEncoder1() {
  int b = digitalRead(enc_b_pins[0]);
  if (b > 0) {
    posi[0]--;
  } else {
    posi[0]++;
  }
}

void readEncoder2() {
  int b = digitalRead(enc_b_pins[1]);
  if (b > 0) {
    posi[1]++;
  } else {
    posi[1]--;
  }
}

void readEncoder3() {
  int b = digitalRead(enc_b_pins[2]);
  if (b > 0) {
    posi[2]++;
  } else {
    posi[2]--;
  }
}

void readEncoder4() {
  int b = digitalRead(enc_b_pins[3]);
  if (b > 0) {
    posi[3]--;
  } else {
    posi[3]++;
  }
}
