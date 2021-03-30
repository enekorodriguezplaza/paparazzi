//
// Created by anish on 14-03-21.
//

//Include  the header and related files
#include "ground_detector.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
#include "modules/computer_vision/ground_detection_testing.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"

//Define navigation and prints
#define NAV_C 
#define GROUND_DETECTOR_VERBOSE TRUE
#define PRINT(string,...) fprintf(stderr, "[ground_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GROUND_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

//Declar variables as given by the computer vision algorithm
volatile int go_no_go;              //0: obstacle, 1: no obstacle
volatile int left_or_right;         // measure of where are there more obstacles (negative on the left, positive on the right)
volatile double confidence_level;   // [0,1]: 0: no confidence at all. 1: great confidence (of no obstacles)

//Declare chooseDirection, written at the bottom of this c file. 
uint8_t chooseDirection(void); //Chooses which direction to rotate upon encountering an obstacle

//Enumerate the possible navigation states
enum navigation_state_t {
  SAFE,                     //bebop is safe
  OBSTACLE_FOUND,           //bebop has found an obstacle
  SEARCH_FOR_SAFE_HEADING,  //upon starting to change direction, loops until is SAFE again
};

// Declare variables used in the periodic function. All of these can be found on the Settings. These determine the navigation behaviour
float GD_MAX_SPEED = 0.6f;               // max flight speed [m/s], taken from Orange Avoider Guided
float GD_HEADING_RATE = RadOfDeg(27.f);  // heading rate [rad/s] at which the bebop turns upon encountering obstacle
float GD_SLOW_FACTOR_FOUND = 0.45f;       //Factor for reduction of velocity when encountering obstacle
float GD_SLOW_FACTOR_SEARCH = 0.45f;      //Factor for reduction of velocity when searching a new heading
float CONFIDENCE_FACTOR = 5.0f;          // Factor to be multiplied with confidence_level to yeild a new velocity.
int RIGHT_THRESHOLD = 4;                 // threshold of left_or_right value to turn right. A bias is added so that it usually turns right when getting out of the cyberzoo

// Define and initialise global variables
enum navigation_state_t navigation_state = SAFE;   // current state in state machine
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]


//Initialisation function
void get_signal_init(void){
    srand(time(NULL));
    chooseDirection(); //already gets a direction of preference (probably right)
}

//Periodic function
void get_signal_periodic(void){

    //VERBOSE_PRINT("%d \n", go_no_go); /*This prints the go_no_go variable in paparazzi. This variable is 1 if the drone
    //                                    can fly straight ahead and 0 otherwise*/
    //VERBOSE_PRINT("The confidence level is %lf and the left_or_right score is %d", confidence_level, left_or_right);

    //Choose the minimum between the maximum bebop speed and what the bebop thinks is cautious. 
    float speed_sp = fminf(GD_MAX_SPEED, CONFIDENCE_FACTOR*(confidence_level+ 0.05)); //0.05 is added such that there is still a velocity, otherwise would just stop when confidence_level=0


    switch (navigation_state){
        case SAFE:                                          //assumes a safe state
            if (go_no_go==0){
                navigation_state = OBSTACLE_FOUND;          //if go_no_go=0, obstacle has been found
            } else {
            guidance_h_set_guided_body_vel(speed_sp, 0);    //else continue at the same speed as before
            }
            break;
        case OBSTACLE_FOUND:

            guidance_h_set_guided_body_vel(GD_SLOW_FACTOR_FOUND*speed_sp, 0); //slow down with another "slowing down factor"
            chooseDirection();                                                // chooses new direction to go to
            navigation_state = SEARCH_FOR_SAFE_HEADING;                       //Once new direction has been chosen, state is searching for heading

            break;  
        case SEARCH_FOR_SAFE_HEADING:
            //New navigation function, can be found in guidance.c 
            guidance_h_nav_new((avoidance_heading_direction * GD_HEADING_RATE), GD_SLOW_FACTOR_SEARCH*speed_sp, 0); //Rotates + continues advancing to be more efficient
            
            //It rotates + advances until the state is SAFE again, in a loop
            if (go_no_go ==1){    
                guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
                navigation_state = SAFE;
            }   
            break;
        default:
            break;

    }
    return;
}

uint8_t chooseDirection(void) //Chooses direction to rotate to
{
  // Chooses which direction to rotate towards
  if (left_or_right <= RIGHT_THRESHOLD ) { // left or right is negative (with bias): obstacle is on the left. A bias is added to go to the right more often
    
    avoidance_heading_direction = 1.f; 	   // if obstacle on the left, turn right
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * GD_HEADING_RATE); //this is only a print, the real thing is in SEARCH_FOR_NEW_HEADING
  
  } else {                                  // obstacle on the right
    avoidance_heading_direction = -1.f;    // if obstacle on the right, turn left
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * GD_HEADING_RATE);
  } 
  return false;
}