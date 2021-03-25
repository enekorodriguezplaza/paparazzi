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

//This one is for the git boys

//These are initialised as zero but using the image_width_printer_init they are changed to the
//proper settings for a green filter
uint8_t lum_min = 0;
uint8_t lum_max = 0;
uint8_t cb_min  = 0;
uint8_t cb_max  = 0;
uint8_t cr_min  = 0;
uint8_t cr_max  = 0;

//TODO: make either boolean or integer with less bits
volatile int go_no_go;

//Integer that will be negative if obstacle on the right and negative if obstacle on the right
volatile int left_or_right;

//double <1 that is higher the clearer the drones path is
volatile double confidence_level;

//Remember image is rotated. Width is longer of the two dimensions
//Length of the rectangle most to the left as percentage of image length (height)
double BOTTOM_LENGTH_PERCENTAGE = 0.5;
//Length of the rectangle most to the right as percentage of image length (height)
double TOP_LENGTH_PERCENTAGE = 0.25;
//How far to the left the rectangles start as a percentage of image width
double BOTTOM_WIDTH_PERCENTAGE = 0.13;
//How far to the left the rectangles extend as a percentage of image width
double TOP_WIDTH_PERCENTAGE = 0.75;
//#width in pixels of each rectangle
int WIDTH_RECT = 5;
//length of each square (the squares are the rectangles in each column. They do not have to be squares)
int LENGTH_SQUARE = 5;
//Every SQUARES_CHECKEDth square is checked for greer
int SQUARES_CHECKED = 2;

int check_for_green(struct image_t *img, int right_corner_row, int right_corner_column, int rect_length) {
    //printf("Working\n");

    //pointer to buffer where image is stored
    uint8_t *buffer = img->buf;

    //Go trough the pixels in the rectangle
    //TODO: Check squares within the rectangles to know if the object is on the left or on the right

    for (uint16_t y = right_corner_row;y <= right_corner_row + rect_length ; y++){
        //This now goes trough a certain percentage pf the image
        //for (uint16_t x = (1-percent_w)*0.5* img->w; x < (1-((1-percent_w)*0.5))* img->w; x ++)
        for (uint16_t x = right_corner_column ; x >= right_corner_column - WIDTH_RECT; x--) {
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
            if (x<img->w){
                printf("The coordinates are row %d and column %d \n",y,x);
                printf("The colour is y: %d u: %d and v: %d \n", *yp, *up, *vp);
            }
            if (!((*yp >= lum_min) && (*yp <= lum_max) &&
                (*up >= cb_min) && (*up <= cb_max) &&
                (*vp >= cr_min) && (*vp <= cr_max)) ){
                return 0;
            }
            else{
                *yp = 255;  // make pixel brighter in image
            }

        }
        }
    return 1;

}

struct image_t *get_rect(struct image_t *img){ //In this function we want to look at the amount of green pixels in a given rectangle


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
                //TODO: add else to add point to left right score if square comes back as not green
                //Check if this rectangle is completely green and if so we are good to go straight ahead

                if (check_for_green(img, right_corner_row, right_corner_column, LENGTH_SQUARE) == 0) {

                    //Since green has been detected set only_green_in_row to 0
                    only_green_in_row = 0;

                    //TODO:add more in depth weights depending on distance of square from center line

                    /*TODO: Check if having a left_or_right variable that constantly changes has any
                     * effect on the performance of the code and if so add a final_left_or_right variable */
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