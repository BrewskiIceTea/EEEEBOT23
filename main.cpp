// Include files for required libraries
#include <stdio.h>
#include <iostream>
//#include "opencv2/core.hpp"
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
            lowH = 148;
            highH = 167;
            lowS = 36;
            highS = 255;
            lowV = 0;
            highV = 180;
            break;

    }
}

void symbolRecognition(Mat frame){
    cv::Point2f dstPoints[4] {
        cv::Point(0,0),
        cv::Point(320,0),
        cv::Point(0, 240),
        cv::Point(320,240)
    };

    Mat frameCShift = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    cvtColor(frame, frameCShift, COLOR_BGR2HSV);
    inRange(frameCShift, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameCShift);
    printf("1");
    std::vector<std::vector<cv::Point>>contours;
    std::vector<cv::Vec4i> hierarchy;
    Mat out = Mat::zeros(frame.rows, frame.cols, CV_8UC1);

    findContours(frameCShift, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    int biggestContour = 0;
    std::vector< std::vector<cv::Point> > approxedcontours(contours.size()); //
    for(int i = 0; i < contours.size(); i++){
        cv::approxPolyDP(contours[i], approxedcontours[i], 5, true); //Approximate the contour
        if (cv::contourArea(approxedcontours[i]) > cv::contourArea(approxedcontours[biggestContour])){
            biggestContour = i;
        }
    }

    cv::Mat srcPointsMat(approxedcontours[biggestContour]);
    cv::Mat dstPointsMat(4, 1, CV_32FC2, dstPoints);
    cv::Mat tfmx = cv::getPerspectiveTransform(srcPointsMat, dstPointsMat);


    cv::Mat out2;
    cv::warpPerspective(frameCShift, out2, tfmx, frame.size());

    drawContours(out, approxedcontours, -1, Scalar(255,255,255), 2);


    imshow("Photo", frameCShift); //Display the image in the window
    imshow("pre",out);
    imshow("Result", out2);

}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices
    cv::namedWindow("Photo");   // Create a GUI window called photo
    namedWindow("Result");
    namedWindow("pre");
    createTrackbar("Mode", "Photo", &mode, 5, NULL);


    while(1)    // Main loop to perform image processing
    {
        Mat frame;

        frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        flip(frame, frame, 0);
        flip(frame, frame, 1);
        imshow("Photo",frame);
        mode = getTrackbarPos("Mode", "Photo");
        changeColour(mode);

        Mat frameCShift = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
        cvtColor(frame, frameCShift, COLOR_BGR2HSV);
        inRange(frameCShift, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameCShift);
        imshow("Result", frameCShift);
        symbolRecognition(frame);
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

	closeCV();  // Disable the camera and close any windows

	return 0;
}



