/*
 * MotorKnob
 *
 * A stepper motor follows the turns of a potentiometer
 * (or other sensor) on analog input 0.
 *
 * http://www.arduino.cc/en/Reference/Stepper
 * This example code is in the public domain.
 */

#include <Stepper.h>
//______________________________________________ Hardware definitions __________________________________________________  //

#define SERIAL_ON 1


#define STEPS 200
#define SPEED 4000
#define STEP_SIZE 1

// Motor control pins
#define MOTOR1_0 2
#define MOTOR1_1 3
#define MOTOR2_1 4
#define MOTOR2_0 5
#define MOTOR3_0 6
#define MOTOR3_1 7
#define MOTOR4_0 8
#define MOTOR4_1 9
#define MOTOR5_0 10
#define MOTOR5_1 11

// motor isolation pins
#define M0M1 13
#define M2 12   
#define M3 A5
#define M4 A4

// EMG definitions
#define S0_TENSE 575
#define S1_TENSE 575

#define S0_FLEX_MIN 175
#define S0_FLEX_MAX 225
#define S1_FLEX_MIN 175
#define S1_FLEX_MAX 225

#define S0_LIGHT_MIN 300
#define S0_LIGHT_MAX 450
#define S1_LIGHT_MIN 300
#define S1_LIGHT_MAX 450

// EMG definitions #signals
#define S1_ON_THRESHOLD 250
#define GESTURE_CYCLES 500
#define STEPSPERLOOP 30000

#define ABS(a) ((a) >= 0 ? (a) : -(a))
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define int int32_t

// Motor objects
//Stepper stepper0(STEPS, MOTOR1_0, MOTOR1_1)  ;
//Stepper stepper1(STEPS, MOTOR2_0, MOTOR2_1)  ;
//Stepper stepper2(STEPS, MOTOR3_0, MOTOR3_1)  ;
//Stepper stepper3(STEPS, MOTOR4_0, MOTOR4_1)  ;
//Stepper stepper4(STEPS, MOTOR5_0, MOTOR5_1)  ;
int motor_pins[5] = {M0M1, M0M1, M2, M3, M4};

//______________________________________________ Control variables and definitions ________________________________________  //

enum hand_state{rest, tripod, grip, drop};
enum finger{little, ring, middle, index, thumb}; // indexes of these enums are the same as the motor that controls that finger

// TODO - Test/calibrate these values
// Range of motion in terms of step numbers for each motor to complete a full actuation
int STEPPER_1_MIN = 0;
int STEPPER_1_MAX = 10000;

int STEPPER_2_MIN = 0;
int STEPPER_2_MAX = 10000;

int STEPPER_3_MIN = 0;
int STEPPER_3_MAX = 10000;

int STEPPER_4_MIN = 0;
int STEPPER_4_MAX = 10000;

int STEPPER_5_MIN = 0;
int STEPPER_5_MAX = 10000;

// Pre-determined gesture postions --- Values are percentage actuation {little, ring, middle, index, thumb}
double REST[5]   = {50, 50, 50, 50, 50};
double TRIPOD[5] = {100, 100, 75, 25, 50};
double GRIP[5]   = {80, 80, 80, 80, 80};
double DROP[5]   = {0, 0, 0, 0, 0};

double* grip_defs[4] = {REST, TRIPOD, GRIP, DROP};

// Current state of each finger in the hand
int hand_position[5] = {0, 0, 0, 0, 0};

// average recent EMG values
double past_emg[2] = {0,0};
int num_emg = 0;
int emg_reset = 5000;
int sensor_on = 0;
int actuation_count = 0;

class Hand {
  public:
  Stepper motors[5] = {Stepper(STEPS, MOTOR1_0, MOTOR1_1), 
                    Stepper(STEPS, MOTOR2_0, MOTOR2_1), 
                    Stepper(STEPS, MOTOR3_0, MOTOR3_1), 
                    Stepper(STEPS, MOTOR4_0, MOTOR4_1), 
                    Stepper(STEPS, MOTOR5_0, MOTOR5_1)};
  int motor_pos[5];
  void init();
  void step(int num0, int num1, int num2, int num3, int num4);
  void stepOne(int i, int num);
  void stepTwo(int i0, int i1, int num0, int num1);
  void addMotorPos(int i, int step);
  void setMotorPos(int i, int step);
};

Hand hand;
//______________________________________________ Control Functions  ________________________________________________________  //

/*
void move_hand(hand_state move_to) {
  int j = 0;  
  int s1 = analogRead(A3);
  if(move_to == grip) {
    while(s1 > S1_ON_THRESHOLD){
      s1 = analogRead(A3);
      for (int i = little; i < thumb; i++) {
        // enable the motor and actuate the finger
       digitalWrite(motor_pins[i], HIGH);
       for (j = 0; j < STEPSPERLOOP; j++) {
          motors[i].step(STEP_SIZE);
       }
       digitalWrite(motor_pins[i], LOW); // deactivate the motor
      }
    }
  }
  if(move_to == tripod) {
    while(s1 > S1_ON_THRESHOLD){
      s1 = analogRead(A3);
      for (int i = little; i < thumb; i++) {
        // enable the motor and actuate the finger
       digitalWrite(motor_pins[i], HIGH);
       for (j = 0; j < STEPSPERLOOP; j++) {
          motors[i].step(-STEP_SIZE);
       }
       digitalWrite(motor_pins[i], LOW); // deactivate the motor
      }
    }
  }

}

void basic_emg(void) {
    int s0 = analogRead(A2);
    int s1 = analogRead(A3);
    //Serial.println(s0);
    //Serial.println(s1);

    // Coded method
    if(s1 > S1_ON_THRESHOLD || sensor_on || num_emg != 1) {
      if(!sensor_on && s1 > S1_ON_THRESHOLD) {
        sensor_on = 1;
      } else if(sensor_on && s1 < S1_ON_THRESHOLD) {
        actuation_count++;
        sensor_on = 0;
      }
      num_emg++;
    }

    if (num_emg >= emg_reset) {

        hand_state state;
   
        if (actuation_count == 1) {
          // Grip pattern
          if (SERIAL_ON) {
            Serial.println("Registered a Grip input.");
          }
          state = grip;
          move_hand(state);
          
        } else if ( actuation_count > 2) {
          // Flex pattern (drop trigger (fully opens and returns to rest))
          if (SERIAL_ON) {
            Serial.println("Registered a halt input.");
          }
          //state = drop;
          //move_hand(state);
          state = rest;
          move_hand(rest);
          
        } else if ( actuation_count == 2) {
          // Tripod trigger
          if (SERIAL_ON) {
            Serial.println("Registered a open input.");
          }
          state = tripod;
          move_hand(state);
        } 
        num_emg = 1;
        actuation_count = 0;
        sensor_on = 0;
    }
    
}
*/

void massert(int cond) {
  if(!cond) {
    while(1) {
      Serial.println("fk!?");
      delay(1000);
    }
  }
}

void Hand::step(int num0, int num1, int num2, int num3, int num4) {
  stepTwo(0, 1, num0, num1);
  stepTwo(3, 4, num3, num4);
  stepOne(2, num2);
}

void Hand::stepOne(int i, int num) {
  int nve = num < motor_pos[i] ? -1 : 1;
  digitalWrite(motor_pins[i], HIGH);
  for(int k = 0; k < ABS(motor_pos[i] - num); k++) {
    motors[i].step(nve * STEP_SIZE);
  }
  setMotorPos(i, num);
  digitalWrite(motor_pins[i], LOW);
}

void Hand::stepTwo(int i0, int i1, int num0, int num1) {
  /* Start WTF!? */
  if(num0 != motor_pos[i0] || num1 != motor_pos[i1]) {
    int nve0 = num0 < motor_pos[i0] ? -1 : 1;
    int nve1 = num1 < motor_pos[i1] ? -1 : 1;
    int bothsteps = MIN(ABS(motor_pos[i0] - num0), ABS(motor_pos[i1] - num1));
    digitalWrite(motor_pins[i0], HIGH);
    digitalWrite(motor_pins[i1], HIGH);
    for(int i = 0; i < bothsteps; i++) {
      motors[i0].step(nve0 * STEP_SIZE);
      motors[i1].step(nve1 * STEP_SIZE);
    }
    addMotorPos(i0, nve0 * bothsteps);
    addMotorPos(i1, nve1 * bothsteps);
    massert(motor_pos[i0] == num0 || motor_pos[i1] == num1);
    //Serial.println(motor_pos[i0]);
    //Serial.println(motor_pos[i1]);
    //Serial.println(num0);
    //Serial.println(num1);
    if(motor_pos[i0] != num0) {
      for(int i = 0; i < ABS(motor_pos[i0] - num0); i++) {
        motors[i0].step(nve0 * STEP_SIZE);
      }
    }
    else {
      for(int i = 0; i < ABS(motor_pos[i1] - num1); i++) {
        motors[i1].step(nve1 * STEP_SIZE);
      }
    }
    setMotorPos(i0, num0);
    setMotorPos(i1, num1);
    digitalWrite(motor_pins[i0], LOW);
    digitalWrite(motor_pins[i1], LOW);
  }
  /* End WTF!? */
}

void Hand::addMotorPos(int i, int step) {
  motor_pos[i] = MAX(motor_pos[i] + step, 0);
}

void Hand::setMotorPos(int i, int step) {
  motor_pos[i] = MAX(step, 0);
}

void Hand::init() {
  pinMode(M0M1, OUTPUT);
  pinMode(M2 ,OUTPUT);
  pinMode(M3 ,OUTPUT);
  pinMode(M4 ,OUTPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  digitalWrite(M0M1, LOW);
  digitalWrite(M2, LOW); 
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
  motors[0].setSpeed(SPEED);
  motors[1].setSpeed(SPEED);
  motors[2].setSpeed(SPEED);
  motors[3].setSpeed(SPEED);
  motors[4].setSpeed(SPEED);
  digitalWrite(motor_pins[0], LOW);
  digitalWrite(motor_pins[1], LOW);
  digitalWrite(motor_pins[2], LOW);
  digitalWrite(motor_pins[3], LOW);
  digitalWrite(motor_pins[4], LOW);

  // TODO Actually set motors to go to zero?
  for (int i = 0; i < 5; i++) {
    motor_pos[i] = 00000;
  }
}

void basic_emg(void) {
    int s0 = analogRead(A2);
    int s1 = analogRead(A3);
    //Serial.println(s0);
    //Serial.println(s1);

    // Coded method
    if(s1 > S1_ON_THRESHOLD || sensor_on || num_emg != 1) {
      if(!sensor_on && s1 > S1_ON_THRESHOLD) {
        sensor_on = 1;
        Serial.println("YES");
      } else if(sensor_on && s1 < S1_ON_THRESHOLD) {
        actuation_count++;
        sensor_on = 0;
      }
      num_emg++;
    }

    if (num_emg >= emg_reset) {
   
        if (actuation_count == 1 || actuation_count == 2) {
          // Grip pattern
          if (SERIAL_ON) {
            Serial.println("Registered a Grip input.");
            hand.step(00000, 70000, 00000, 120000, 100000);
          }
          //state = grip;
          //move_hand(state);
          
        } else if ( actuation_count > 2) {
          // Flex pattern (drop trigger (fully opens and returns to rest))
          if (SERIAL_ON) {
            Serial.println("Registered a halt input.");
            hand.step(00000, 00000, 00000, 00000, 00000);
          }
          //state = drop;
          //move_hand(state);
          //state = rest;
          //move_hand(rest);
          
        }
        num_emg = 1;
        actuation_count = 0;
        sensor_on = 0;
    }
    
}

void setup() {
  if (SERIAL_ON) {
    Serial.begin(9600);
  }

  hand.init();

}

void loop() {
  basic_emg();
  //hand.step(00000, 00000, 00000, 00000, 00000);
  //Serial.println("Hi");

  //delay(2000);
  //hand.step(0, 0, 0, 0, 0);
  //delay(2000);
  //motors[0].step(STEP_SIZE);
  //motors[1].step(STEP_SIZE);
  //motors[2].step(STEP_SIZE);    
  //motors[2].step(-STEP_SIZE);  
  //basic_emg();
}


