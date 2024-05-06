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
#include <chrono>

using namespace cv;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

int lowH, highH, lowS, highS, lowV, highV, mode = 5;


double previous_error; //PID

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

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
    double lastError = 0.0;
    while(1)
    {
        Mat frame, frameHSV, frameBlur, frameEdges;
        frame = captureFrame();
        flip(frame, frame, 0);
        flip(frame, frame, 1);

        // Adding crops to frame to reduce unwanted input and also to close contours.
        Point p1(0,0);
        Point p2(320, 40);
        rectangle(frame, p1, p2,Scalar(0,255,0), -1, LINE_8);

        Point p3(0, 200);
        Point p4(320, 240);
        rectangle(frame, p3, p4,Scalar(0,255,0), -1, LINE_8);

        imshow("Photo",frame);
        changeColour(2);

        // processing
        cvtColor(frame, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), frameHSV);
        cv::medianBlur(frameHSV, frameBlur, 15); // Create blurred image
        cv::Canny(frameBlur, frameEdges, 100, 100*3, 3); // Create edges from image

        // Closing the frame
        int morph_size = 2;
        Mat element = getStructuringElement(
        MORPH_RECT,
        Size(2 * morph_size + 1,
             2 * morph_size + 1),
        Point(morph_size, morph_size));

        morphologyEx(frameEdges, frameEdges,
                     MORPH_CLOSE, element,
                     Point(-1, -1), 2);

        // Contours

        std::vector< std::vector<cv::Point> > contours;
        std::vector<Vec4i> hierarchy;// Variable for image topology data

        cv::findContours(frameEdges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); // Calculate the contours and store them
        std::vector< std::vector<cv::Point> > approxedcontours(contours.size()); //Array for new contours

        for(int i = 0; i < contours.size(); i++)
        {
        //Approximate the contour
        cv::approxPolyDP(contours[i],approxedcontours[i], 3, true);
        }

        Mat approxedContour = frame;


        for(int i = 0; i < contours.size(); i++) // Loop through the contours
        {
            drawContours(approxedContour, approxedcontours, i, Scalar(0,255,0), 3, LINE_4, noArray(), 0, Point() ); // Draw each in red }
        }


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
        // when
        if (!contours.empty()) {
            //Find position of line centre
            int linePosition = *pcx;

            //Drawing centre point
            cv::circle(frame, cv::Point(linePosition, height), 5, cv::Scalar(255, 0, 0), -1); /// Blue dot
            printf("\nline position: \n%d\n",linePosition);
            double setpoint = frameWidth / 2;
            double control_signal = PID(setpoint, linePosition);
            printf("\ncontrol_signal: %f \n", control_signal);
            Pi2c esp32(4); //Create a new object "esp32" using address "0x08"
            //esp32.i2cWriteArduinoInt(5);
            esp32.i2cWriteArduinoInt(83 + int (-control_signal/2.5));
        }

        namedWindow("Original",WINDOW_AUTOSIZE);
        cv::imshow("Result", frameEdges);
        cv::imshow("pre", approxedContour);
        cv::imshow("Original", frame);
        cv::imshow("ApproxContours",approxedContour);


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

int PID(double setpoint, double pv)
{
    // Attributes
    double Kp = 0.5; // Proportional gain
    double Ki = 0.013; // Integral gain
    double Kd = 0.2;  // Derivative gain
    double integral = 0.0;
    double derivative = 0.0;

    double error = setpoint - pv;
    double Pout = Kp * error;

    integral += error;
    double Iout = Ki * integral;

    derivative = error - previous_error;
    double Dout = Kd * derivative;

    double output = Pout + Iout + Dout;

    previous_error = error;

    return output;
}

