#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include<stdio.h>
#include<dos.h>
using namespace cv;

int main(int argc, char *argv[])
{
    //setup();    // Call a setup function to prepare IO and devices

    //cv::namedWindow("Camera");   // Create a GUI window called photo

    while(1)    // Main loop to perform image processing
    {
        Mat frame;

        // Can't capture frames without a camera attached. Use static images instead
        while(frame.empty())
        {
            /// Can't capture frames without a camera attached. Use static images instead
            //frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
            //frame = readImage("OpenCV_Logo.png");
        }
        // Rotate the frame since RPi camera is upside down
        Mat frameRotated;
        Point2f pc(frame.cols/2., frame.rows/2.);
        Mat RotateVect = cv::getRotationMatrix2D(pc, -180, 1.0);
        cv::warpAffine(frame, frameRotated, RotateVect, frame.size());

        cv::imshow("Photo", frameRotated); //Display the image in the window

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}
    */
    while(1)
        {
	Mat frame;
	String image = "C:\\Users\\16mal\\Desktop\\OpenCV Task\\Images\\BlueApple.bmp";
    frame = imread(image);
    //cv::resize(frame, frame, cv::Size(frame.cols * 0.5,frame.rows * 0.5), 0, 0, CV_INTER_LINEAR);

	// Defining different Mat variables
	Mat frameHSV, frameThresh, frameBlur, frameEdges;
    // Convert the image to HSV
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    // Threshold for Blue
    //inRange(frameHSV, Scalar(95, 80, 60), Scalar(120, 255, 255), frameThresh);
    // Threshold for Red
    //inRange(frameHSV, Scalar(0, 170, 140), Scalar(179, 255, 255), frameThresh);
    // Create blurred image
    cv::medianBlur(frameThresh, frameBlur, 15);
    // Create edges from image
    cv::Canny(frameBlur, frameEdges, 100, 100*3, 3);
    // Variable for list of contours
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

    Mat approxedContour = imread(image);
    for(int i = 0; i < contours.size(); i++)
    {
    // Loop through the contours
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

    printf("Control signal: %f", control_signal);

    //double time_taken = ((double)current_time)/CLOCKS_PER_SEC; // in seconds
     //printf("\nTime: %f",current_time);
    //current_time = clock();
    //printf("\nTime: %d",current_time);

    namedWindow("Original",WINDOW_AUTOSIZE);
    cv::imshow("Original", frame);
    cv::imshow("Threshed", frameThresh);
    //cv::resize(approxedContour, approxedContour, cv::Size(frame.cols * 0.5,frame.rows * 0.5), 0, 0, CV_INTER_LINEAR);
    cv::imshow("ApproxContours",approxedContour);
    waitKey();
    }

}

    // Convert the image to HSV
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    switch(line2follow){

    case "r":
    inRange(frameHSV, Scalar(0, 170, 140), Scalar(179, 255, 255), frameThresh);
    break;
    case "g":
    inRange(frameHSV, Scalar(95, 80, 60), Scalar(120, 255, 255), frameThresh);
    break;
    case "b":
    inRange(frameHSV, Scalar(95, 80, 60), Scalar(120, 255, 255), frameThresh);
    break;
    case "y":
    inRange(frameHSV, Scalar(20, 100, 100), Scalar(50, 255, 255), frameThresh);
    break;
    }




