// Include files for required libraries
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv_aee.hpp"
#include "main.hpp"
#include "pi2c.h"
#include <time.h>

using namespace cv;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

int lowH, highH, lowS, highS, lowV, highV, mode = 5;

// PID Constants
float Kp = 0.5; // Proportional gain
float Ki = 0.1; // Integral gain
float Kd = 0.2; // Derivative gain

float previousError = 0.0;
float integral = 0.0;

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}
/*
float PID() {

    float weightedAverage = 0;

    // Calculate weighted average
    for (int i = 0; i < 3; i++) {
        weightedAverage += ((i - 1) * IRarray[i]);
    }



    // threshold value for considering black color
    int targetValue = 3000;

    // calculate the error as the difference between the target and the weighted average
    float error = targetValue - weightedAverage;

    // calculating all terms
    float proportional = Kp * error;
    integral += Ki * error;
    float derivative = Kd * (error - previousError);

    // combine all terms and keep output within servo range
    float output = proportional + integral + derivative;
    int angle = constrain(output, 45, 135);

    // Update the previous error for the next iteration
    previousError = error;

    Serial.println(angle);
    return angle;
}
*/
void changeColour(int colour){
    switch(colour){
        case 0: //red
            lowH = 159;
            highH = 179;
            lowS = 171;
            highS = 255;
            lowV = 0;
            highV = 255;
            break;
        case 1: //green
            lowH = 65;
            highH = 87;
            lowS = 165;
            highS = 255;
            lowV = 0;
            highV = 255;
            break;
        case 2: //blue
            lowH = 95;
            highH = 120;
            lowS = 125;
            highS = 255;
            lowV = 0;
            highV = 255;
            break;
        case 3: //yellow
            lowH = 20;
            highH = 50;
            lowS = 100;
            highS = 255;
            lowV = 100;
            highV = 255;
            break;
        case 4: //black
            lowH = 0;
            highH = 179;
            lowS = 0;
            highS = 255;
            lowV = 0;
            highV = 41;
            break;
        case 5: //pink
            lowH = 145;
            highH = 169;
            lowS = 36;
            highS = 255;
            lowV = 0;
            highV = 180;
            break;

    }
}


int main( int argc, char** argv )
{
    try {
    setup();    // Call a setup function to prepare IO and devices
    cv::namedWindow("Photo");   // Create a GUI window called photo
    namedWindow("Result");
    namedWindow("pre");
    createTrackbar("Mode", "Photo", &mode, 5, NULL);
    printf("hiii");
    cv::Rect myROI(0, 70, 320, 170);
    while(1)    // Main loop to perform image processing
    {
        Mat frame, frameHSV, frameBlur, frameEdges;
        frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        flip(frame, frame, 0);
        flip(frame, frame, 1);
        frame = frame(myROI);
        imshow("Photo",frame);
        changeColour(2);

        Mat frameCShift = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);
        // Threshold for Blue
        //inRange(frameHSV, Scalar(95, 80, 60), Scalar(120, 255, 255), frameThresh);
        // Threshold for Red
        //inRange(frameHSV, Scalar(0, 170, 140), Scalar(179, 255, 255), frameThresh);
        // Create blurred image
        cv::medianBlur(frameHSV, frameBlur, 15);
        // Create edges from image
        cv::Canny(frameBlur, frameEdges, 100, 100*3, 3);
        // Variable for list of contours
        int morph_size = 2;
        Mat element = getStructuringElement(
        MORPH_RECT,
        Size(2 * morph_size + 1,
             2 * morph_size + 1),
        Point(morph_size, morph_size));

        // Closing
        morphologyEx(frameEdges, frameEdges,
                     MORPH_CLOSE, element,
                     Point(-1, -1), 2);
        imshow("Result", frameEdges);
        std::vector< std::vector<cv::Point> > contours;
        // Variable for image topology data
        std::vector<Vec4i> hierarchy;
        // Calculate the contours and store them
        cv::findContours(frameEdges, contours, hierarchy, RETR_TREE,CHAIN_APPROX_SIMPLE, Point(0, 0));
        //Array for new contours
        std::vector< std::vector<cv::Point> > approxedcontours(contours.size());

        for(int i = 0; i < contours.size(); i++)
        {
        //Approximate the contour
        cv::approxPolyDP(contours[i],approxedcontours[i], 3, true);
        }

        Mat approxedContour = frame;


        for(int i = 0; i < contours.size(); i++)
        {
        // Loop through the contours
        drawContours(approxedContour, approxedcontours, i, Scalar(0,255,0), 3, LINE_4, noArray(), 0, Point() ); // Draw each in red }
        }
        imshow("pre", approxedContour);

        int* pcx;
        std::vector<cv::Point> centres;
        for (int i = 0; i < contours.size(); i++)
        {
        Moments M = cv::moments(contours[i]);
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        cv::circle(approxedContour, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1); // Red dot
        // Calculate the area of a contour
        int area = cv::contourArea(contours[i]);
        printf("Area of contour = %d\n", area);
        pcx = &cx;
        }
        printf("hii");
        //Get size of frame, width/height
        int frameWidth,frameHeight,centreFrame;
        cv::Size sz = frame.size();
        frameWidth = sz.width;
        centreFrame = frameWidth/2;
        frameHeight = sz.height;
        printf("Width: %d, Height: %d",frameWidth,frameHeight);
        //From left to right
        int gap = frameWidth/5;
        int height = frameHeight/2;
        //Find position of line centre
        int linePosition = *pcx;
        cv::circle(frame, cv::Point(linePosition, height), 5, cv::Scalar(255, 0, 0), -1); /// Blue dot
        printf("\n%d\n",linePosition);
        int centre = centreFrame;
        int centreDist2Line = centre - linePosition;
        cv::circle(frame, cv::Point(centre, height), 5, cv::Scalar(0, 0, 255), -1); /// Red dot

        int left1 = centre - gap*2;
        int left1Dist2Line = left1 - linePosition;
        cv::circle(frame, cv::Point(left1, height), 5, cv::Scalar(0, 0, 255), -1); /// Red dot

        int left2 = centre - gap;
        int left2Dist2Line = left2 - linePosition;
        cv::circle(frame, cv::Point(left2, height), 5, cv::Scalar(0, 0, 255), -1); /// Red dot

        int right2 = centre + gap;
        int right2Dist2Line = right2 - linePosition;
        cv::circle(frame, cv::Point(right2, height), 5, cv::Scalar(0, 0, 255), -1); /// Red dot

        int right1 = centre + gap*2;
        int right1Dist2Line = right1 - linePosition;
        cv::circle(frame, cv::Point(right1, height), 5, cv::Scalar(0, 0, 255), -1); /// Red dot
        //Position
        int pointPosition[5] = {left1,left2,centre,right2,right1};
        //Vec2i = ()
        // "Intensity"
        int pointIntensity[5] = {left1Dist2Line,left2Dist2Line,centreDist2Line,right2Dist2Line,right1Dist2Line};
        //Vec2i =

        //WeightedAverage values
        float numerator,denominator;

        //PID values for calc
          float control_signal = 0;
          int Kp = 3; //proportional gain
          float Ki = 3; //integral gain
          int Kd = 5; //derivative gain
          int T = 10; //sample time in milliseconds (ms)
          unsigned long last_time;
          float total_error, last_error;
    //calculateWeightedAverage

         for (int i = 0; i < 6; i++)
      {
        numerator += (pointPosition[i]*pointIntensity[i]);
        denominator += pointIntensity[i];
      }


      float weightedAverage = (numerator)/(denominator);


    //PID_control
      //clock_t current_time;
      //current_time = clock(); //=
      //current_time = ((double)current_time)/CLOCKS_PER_SEC; // in seconds
      //int delta_time = current_time - last_time; //delta time interval

      //if (delta_time >= T)
        //{
        float error = 0 - weightedAverage;

        total_error += error; //accumalates the error - integral term

        float delta_error = error - last_error; //difference of error for derivative term

        control_signal = Kp*error + (Ki*total_error) + (Kd*delta_error); //PID control compute


        last_error = error;
        //last_time = current_time;
        //}

       // printf("Control signal: %f", control_signal);

        //double time_taken = ((double)current_time)/CLOCKS_PER_SEC; // in seconds
         //printf("\nTime: %f",current_time);
        //current_time = clock();
        //printf("\nTime: %d",current_time);

        namedWindow("Original",WINDOW_AUTOSIZE);
        cv::imshow("Original", frame);
        //cv::resize(approxedContour, approxedContour, cv::Size(frame.cols * 0.5,frame.rows * 0.5), 0, 0, CV_INTER_LINEAR);
        cv::imshow("ApproxContours",approxedContour);



        //symbolRecognition(frameCShift);

        /*for (int i = 0; i <) {
            inRange(frameCShift, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameCShift);
            noPixel()
          //  array[]
        }
        */

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)
        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}
}
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
	return 0;
}



