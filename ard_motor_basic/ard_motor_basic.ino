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
#define SPEED 5000
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


// Motor objects
//Stepper stepper0(STEPS, MOTOR1_0, MOTOR1_1)  ;
//Stepper stepper1(STEPS, MOTOR2_0, MOTOR2_1)  ;
//Stepper stepper2(STEPS, MOTOR3_0, MOTOR3_1)  ;
//Stepper stepper3(STEPS, MOTOR4_0, MOTOR4_1)  ;
//Stepper stepper4(STEPS, MOTOR5_0, MOTOR5_1)  ;
int motor_pins[5] = {M0M1, M2, M3, M4};
Stepper motors[5] = {Stepper(STEPS, MOTOR1_0, MOTOR1_1), 
                    Stepper(STEPS, MOTOR2_0, MOTOR2_1), 
                    Stepper(STEPS, MOTOR3_0, MOTOR3_1), 
                    Stepper(STEPS, MOTOR4_0, MOTOR4_1), 
                    Stepper(STEPS, MOTOR5_0, MOTOR5_1)};

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
int emg_reset = 500; 
//______________________________________________ Control Functions  ________________________________________________________  //

void move_hand(hand_state move_to) {

  for (int i = little; i < thumb; i++) {

      if (grip_defs[move_to][i] > hand_position[i]) {
          // enable the motor and actuate the finger
          digitalWrite(motor_pins[i], HIGH); 
          while (grip_defs[move_to][i] > hand_position[i]) {

              hand_position[i] += STEP_SIZE;
              motors[i].step(STEP_SIZE);
          }
          digitalWrite(motor_pins[i], LOW); // deactivate the motor
      
      } else if (grip_defs[move_to][i] < hand_position[i]) {
          // enable the motor and actuate the finger
          digitalWrite(motor_pins[i], HIGH); 
          while (grip_defs[move_to][i] < hand_position[i]) {

              hand_position[i] -= STEP_SIZE;
              motors[i].step(STEP_SIZE);
          }
          digitalWrite(motor_pins[i], LOW); // deactivate the motor
      }
  }
}

void basic_emg(void) {
    int s0 = analogRead(A2);
    int s1 = analogRead(A3);
    
    // We average the emg readings until emg_reset is met, then make a decision
    past_emg[0] = (past_emg[0] * num_emg + s0)/(num_emg + 1);
    past_emg[1] = (past_emg[1] * num_emg + s1)/(num_emg + 1);
    num_emg ++;
    
    if (num_emg >= emg_reset) {

        hand_state state;
   
        if (past_emg[0] > S0_TENSE && past_emg[1] > S1_TENSE) {
          // Grip pattern
          if (SERIAL_ON) {
            Serial.println("Registered a Grip input.");
          }
          state = grip;
          move_hand(state);
          
        } else if ( past_emg[0] > S0_FLEX_MIN && past_emg[0] < S0_FLEX_MAX && 
                    past_emg[1] > S1_FLEX_MIN && past_emg[1] < S1_FLEX_MAX) {
          // Flex pattern (drop trigger (fully opens and returns to rest))
          if (SERIAL_ON) {
            Serial.println("Registered a drop input.");
          }
          state = drop;
          move_hand(state);
          state = rest;
          move_hand(rest);
          
        } else if ( past_emg[0] > S0_LIGHT_MIN && past_emg[0] < S0_LIGHT_MAX && 
                    past_emg[1] > S1_LIGHT_MIN && past_emg[1] < S1_LIGHT_MAX) {
          // Tripod trigger
          if (SERIAL_ON) {
            Serial.println("Registered a tripod input.");
          }
          state = tripod;
          move_hand(state);
        } 
        num_emg = 1;
    }
}

void setup() {

  if (SERIAL_ON) {
    Serial.begin(9600);
  }
  
  // set the speed of the motor to 30 RPMs
  motors[0].setSpeed(SPEED);
  motors[1].setSpeed(SPEED);
  motors[2].setSpeed(SPEED);
  motors[3].setSpeed(SPEED);
  motors[4].setSpeed(SPEED);
  
  pinMode(M0M1 ,OUTPUT);
  pinMode(M2 ,OUTPUT);
  pinMode(M3 ,OUTPUT);
  pinMode(M4 ,OUTPUT);

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  
  digitalWrite(M0M1, LOW);
  digitalWrite(M2, LOW); 
  digitalWrite(M3, LOW);
  digitalWrite(M4, HIGH);
  
  

}

void loop() {

 basic_emg();

}


