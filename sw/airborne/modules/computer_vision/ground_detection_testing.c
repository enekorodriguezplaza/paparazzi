//
// Created by anish on 17-03-21.
//

#include "ground_detection_testing.h"
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#ifndef FPS_WIDTH
#define FPS_WIDTH 0
#endif

//Trying to push to a new branch

//These are initialised as zero but using the image_width_printer_init they are changed to the
//proper settings for a green filter
uint8_t lum_min = 0;
uint8_t lum_max = 0;
uint8_t cb_min  = 0;
uint8_t cb_max  = 0;
uint8_t cr_min  = 0;
uint8_t cr_max  = 0;

volatile int go_no_go;

//Integer that will be negative if obstacle on the right and negative if obstacle on the right
volatile int left_or_right;

//double <1 that is higher the clearer the drones path is
volatile double confidence_level;

int INITIALISE_GREEN = 0;

//Remember image is rotated. Width is longer of the two dimensions
//Length of the rectangle most to the left as percentage of image length (height)
double BOTTOM_LENGTH_PERCENTAGE = 0.5;
//Length of the rectangle most to the right as percentage of image length (height)
double TOP_LENGTH_PERCENTAGE = 0.25;
//How far to the left the rectangles start as a percentage of image width
double BOTTOM_WIDTH_PERCENTAGE = 0.1;
//How far to the left the rectangles extend as a percentage of image width
double TOP_WIDTH_PERCENTAGE = 0.75;
//Width in pixels of each rectangle
int WIDTH_RECT = 5;
//length of each square (the squares are the rectangles in each column. They do not have to be squares)
int LENGTH_SQUARE = 8;
//Every SQUARES_CHECKEDth square is checked for greer
int SQUARES_CHECKED = 2;


int green_initialised = 0;

double bottom_green_length_percentage = 0.25;
double top_green_length_percentage = 0.75;
double bottom_green_width_percentage = 0.2;
double top_green_width_percentage = 0.4;

float lum_min_coeff = 5.5;
float lum_max_coeff = 5.5;
float cb_min_coeff  = 3.5;
float cb_max_coeff  = 3.5;
float cr_min_coeff  = 3.5;
float cr_max_coeff = 3.5;

int check_for_green(struct image_t *img, int right_corner_row, int right_corner_column) {
    //printf("Working\n");

    //pointer to buffer where image is stored
    uint8_t *buffer = img->buf;

    uint32_t tot_lum;
    uint32_t tot_cb;
    uint32_t tot_cr;

    //Go through the pixels in the rectangle
    for (uint16_t y = right_corner_row;y < right_corner_row + LENGTH_SQUARE; y++) {
        //This now goes trough a certain percentage pf the image
        //for (uint16_t x = (1-percent_w)*0.5* img->w; x < (1-((1-percent_w)*0.5))* img->w; x ++)
        for (uint16_t x = right_corner_column; x > right_corner_column - WIDTH_RECT; x--) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;

            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }

            //Add pixels YUV values to their respective totals
            tot_lum += *yp;
            tot_cb += *up;
            tot_cr += *vp;
        }
    }

    //Take average of the YUV values of the box
    double avg_lum = tot_lum/(LENGTH_SQUARE*WIDTH_RECT);
    double avg_cb = tot_cb/(LENGTH_SQUARE*WIDTH_RECT);
    double avg_cr = tot_cr/(LENGTH_SQUARE*WIDTH_RECT);

    //printf("The average Y value is %lf \n", avg_lum);
    //printf("The average U value is %lf \n", avg_cb);
    //printf("The average V value is %lf \n", avg_cr);

    //If this average is outside the bounds make the pixel lighter and return 0
    if (!((avg_lum >= lum_min) && (avg_lum <= lum_max) &&
        (avg_cb >= cb_min) && (avg_cb <= cb_max) &&
        (avg_cr>= cr_min) && (avg_cr <= cr_max)) ){

        uint8_t *yp;
        yp = &buffer[right_corner_row * 2 * img->w + 2 * right_corner_column + 1];
        *yp = 255;  // make pixel brighter in image
        return 0;
    }
    //If the square is within bounds return 1
    return 1;

}

void init_green(struct image_t *img) {
    //This function is to initialise the colour filter upper and lower bounds irrespective of lighting conditions


    //pointer to buffer where image is stored
    uint8_t *buffer = img->buf;

    //create arrays of integers that will contain the YUV values of all pixels considered
    int lum_values[(int)((top_green_length_percentage-bottom_green_length_percentage)*img->w)*(int)((top_green_width_percentage-bottom_green_width_percentage)*img->h)];
    int cb_values[(int)((top_green_length_percentage-bottom_green_length_percentage)*img->w)*(int)((top_green_width_percentage-bottom_green_width_percentage)*img->h)];
    int cr_values[(int)((top_green_length_percentage-bottom_green_length_percentage)*img->w)*(int)((top_green_width_percentage-bottom_green_width_percentage)*img->h)];

    int counter = 0;

    //Iterate through only the area that should have been cleared of obstacles
    for (int y = floor((bottom_green_length_percentage * img->h)); y < floor((top_green_length_percentage * img->h)); y++) {
        for (int x = floor((bottom_green_width_percentage * img->w)); x < floor((top_green_width_percentage * img->w)); x++){

            uint8_t *yp, *up, *vp;
            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }

            lum_values[counter] = *yp;
            cb_values[counter] = *up;
            cr_values[counter] = *vp;

            counter++;
        }
    }

    //initialise variables for the sum, mean and standard deviation of all the detected YUV values
    int lum_sum = 0;
    float lum_mean;
    float lum_std_dev = 0.0;

    int cb_sum = 0;
    float cb_mean;
    float cb_std_dev = 0.0;

    int cr_sum = 0;
    float cr_mean;
    float cr_std_dev = 0.0;

    //sum all values in each array to get the sum
    for (int i = 0; i<counter; i++){
        lum_sum += lum_values[i];
        cb_sum += cb_values[i];
        cr_sum += cr_values[i];
    }

    //the mean is the sum divided by the number of items in each array
    lum_mean = lum_sum/counter;
    cb_mean = cb_sum/counter;
    cr_mean = cr_sum/counter;

    //Find difference of each point in array with the mean and square this
    for (int i = 0; i<counter; i++){
        lum_std_dev += pow(lum_values[i]-lum_mean,2);
        cb_std_dev += pow(cb_values[i]-cb_mean,2);
        cr_std_dev += pow(cr_values[i]-cr_mean,2);
    }

    //take the square root of the average difference as the standard deviation
    lum_std_dev = sqrt(lum_std_dev/counter);
    cb_std_dev = sqrt(cb_std_dev/counter);
    cr_std_dev = sqrt(cr_std_dev/counter);

    //define the upper and lower bounds based on the mean, standard deviation and coefficients found experimentally
    //for each bound
    lum_min = (int)(floor((lum_mean-lum_min_coeff*lum_std_dev)));
    lum_max = (int)(ceil((lum_mean+lum_max_coeff*lum_std_dev)));
    cb_min  = (int)(floor((cb_mean-cb_min_coeff*cb_std_dev)));
    cb_max  = (int)(ceil((cb_mean+cb_max_coeff*cb_std_dev)));
    cr_min  = (int)(floor((cr_mean-cr_min_coeff*cr_std_dev)));
    cr_max  = (int)(ceil((cr_mean+cr_max_coeff*cr_std_dev)));

    //print upper and lower bounds
    printf("lum_min (Y) is %d\n", lum_min);
    printf("lum_max (Y) is %d\n", lum_max);
    printf("cb_min (U) is %d \n", cb_min);
    printf("cb_max (U) is %d \n", cb_max);
    printf("cr_min (V) is %d \n", cr_min);
    printf("cr_max (V) is %d \n", cr_max);

    return;
}

struct image_t *get_rect(struct image_t *img){ //In this function we want to look at the amount of green pixels in a given rectangle

    //If the green filter hasn't been initialised yet green_initialised will be 0
    if (green_initialised == 0){
        //INITIALISE_GREEN will be 0 till the setting has been manually changed which should only be done once the area shown in the
        //rectangle on the green is completely clear of obstacles and only contains grass pixels
        if (INITIALISE_GREEN == 0) {

            printf("Please make sure area highlighted in drone video stream is \n ONLY the grass of the CyberZoo and then set INITIALISE_GREEN to 1\n");

            uint8_t *buffer = img->buf;

            //Draw rec
            for (int y = floor((bottom_green_length_percentage * img->h)); y < floor((top_green_length_percentage * img->h)); y++) {
                for (int x = floor((bottom_green_width_percentage * img->w)); x < floor((top_green_width_percentage * img->w)); x++){
                    uint8_t *yp, *up, *vp;
                    if (x % 2 == 0) {
                        // Even x
                        //up = &buffer[y * 2 * img->w + 2 * x];      // U
                        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                        //vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
                    } else {
                        // Uneven x
                        //up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                        //vp = &buffer[y * 2 * img->w + 2 * x];      // V
                        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
                    }
                    //make the rectangle lighter on video stream
                    *yp = 255;

                }
            }
            return img;
        }
        init_green(img);
        green_initialised = 1;
        printf("Green has been initialised!\n");
    }

  int rows = img->h;
  int columns = img->w;


  //The number of rectangles which are considered. It is set so that the space
  //up to TOP_WIDTH_PERCENTAGE is filled with non-overlapping rectangles
  int num_rect = ceil((TOP_WIDTH_PERCENTAGE-BOTTOM_WIDTH_PERCENTAGE)*columns/WIDTH_RECT);

  //The difference in length between 2 adjacent rectangles in percentage of image length (height)
  double rect_length_increment = (BOTTOM_LENGTH_PERCENTAGE-TOP_LENGTH_PERCENTAGE)/num_rect;

  //initialise only_green_in_row which will be 1 if a row has no green and 0 otherwise
  int only_green_in_row;

  //initialise left_or_right to 0
  left_or_right = 0;

  //initialise prev_left_right:
  int prev_left_right;

  //Go through rectangles starting from smallest (most to the right)
  for (int rect_num = 0; rect_num < num_rect; rect_num++) {
      //This makes sure that we can use the last checked rectangle, with an obstacle, to see if
      //that obstacle is left or right. It contains the left_or_right score of the previous rectangle
      prev_left_right = left_or_right;

      left_or_right = 0; //The "score" if an obstacle is left or right is reset to 0 for each new rectangle

      //initialise only_green_in_row which will be 1 if a row has no green
      only_green_in_row = 1;

      //The length of the rectangle for this iteration pixels
      int rect_length = floor((TOP_LENGTH_PERCENTAGE + rect_num * rect_length_increment)*rows);

      //This is the number of squares in each row
      int num_squares = ceil(rect_length/LENGTH_SQUARE);


      for (int square_num = 0; square_num < num_squares; square_num++) {

          //Coordinates of right corner
          int right_corner_row = 0.5*(rows - rect_length) + LENGTH_SQUARE*square_num;
          int right_corner_column = TOP_WIDTH_PERCENTAGE * columns - WIDTH_RECT * rect_num;


          if (square_num%SQUARES_CHECKED == 0){
              //Check if this rectangle is completely green and if so we are good to go straight ahead

              if (check_for_green(img, right_corner_row, right_corner_column) == 0) {

                  //Since green has been detected set only_green_in_row to 0
                  only_green_in_row = 0;

                    //if square is to the left of the center of the image subtract 1 from left_or_right
                    if (right_corner_row+0.5*LENGTH_SQUARE < 0.5*rows){
                        left_or_right--; //For objects on the left
                    }
                    //else add 1 to left_or_right
                    else{
                        left_or_right++; //For objects on the right
                    }

                }
            }
        } //End of square for loop


        //if a rectangle has only green
        if (only_green_in_row == 1) {
            left_or_right = prev_left_right;

            //set confidence level to decrease the closer the only green rectangle to the drone
            confidence_level = (double)(num_rect-rect_num)/num_rect;
            go_no_go = 1;
            return img;


        }




    } //End of rectangle for loop
    left_or_right = prev_left_right;

    confidence_level = 0.0;
    go_no_go = 0;
    return img;

    //go_no_go = check_for_green(img,PERCENT_WIDTH_RECT_1,PERCENT_HEIGHT_RECT_1);

}

void image_width_printer_init(void) {


#ifdef WIDTH_CAMERA
    //Here we define the filter settings for the check_for_green function, all the uppercase variables
    //are defined in bebop_ground_detector and they have different values for the ap mode and nps mode
    #ifdef GROUND_DETECTOR_LUM_MIN
  lum_min = GROUND_DETECTOR_LUM_MIN;
  lum_max = GROUND_DETECTOR_LUM_MAX;
  cb_min = GROUND_DETECTOR_CB_MIN;
  cb_max = GROUND_DETECTOR_CB_MAX;
  cr_min = GROUND_DETECTOR_CR_MIN;
  cr_max = GROUND_DETECTOR_CR_MAX;
#endif
    cv_add_to_device(&WIDTH_CAMERA, get_rect, FPS_WIDTH);
#endif
}