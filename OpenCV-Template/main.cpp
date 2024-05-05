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
using namespace std;

int minHessian = 400;

// Function to perform feature detection and matching
void featureMatching(const Mat& img1, const Mat& img2, const string& templateName, int& maxMatches, string& bestTemplate, Mat& bestImgMatches) {
    // Initialize feature detector (SURF)
    Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    Ptr<DescriptorExtractor> extractor = cv::xfeatures2d::SURF::create();

    // Detect keypoints and extract descriptors for the first image
    vector<KeyPoint> keypoints_img1;
    Mat descriptors_img1;
    detector->detect(img1, keypoints_img1);
    extractor->compute(img1, keypoints_img1, descriptors_img1);

    // Detect keypoints and extract descriptors for the second image
    vector<KeyPoint> keypoints_img2;
    Mat descriptors_img2;
    detector->detect(img2, keypoints_img2);
    extractor->compute(img2, keypoints_img2, descriptors_img2);

    // Match descriptors
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match(descriptors_img1, descriptors_img2, matches);

    // Filter matches based on distance
    double max_dist = 0;
    double min_dist = 100;
    for (int i = 0; i < descriptors_img1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_img1.rows; i++) {
        if (matches[i].distance <= max(3 * min_dist, 0.1)) { // Adjust the threshold as needed
            good_matches.push_back(matches[i]);
        }
    }

    // Print number of good matches
    cout << "Number of good matches for " << templateName << ": " << good_matches.size() << endl;

    // Update maximum matches and corresponding template if current template has more matches
    if (good_matches.size() > maxMatches) {
        maxMatches = good_matches.size();
        bestTemplate = templateName;

        // Draw matches
        Mat img_matches;
        drawMatches(img1, keypoints_img1, img2, keypoints_img2, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Store the best image matches
        bestImgMatches = img_matches.clone();
    }
}

int main() {
    // Paths to templates
    vector<string> templates;
    templates.push_back("C:/Users/autsj/OneDrive - The University of Nottingham/lab/report/Report 3/Circle (Red Line)(1).png");
    templates.push_back("C:/Users/autsj/OneDrive - The University of Nottingham/lab/report/Report 3/Star (Green Line)(1).png");
    templates.push_back("C:/Users/autsj/OneDrive - The University of Nottingham/lab/report/Report 3/Triangle (Blue Line).png");
    templates.push_back("C:/Users/autsj/OneDrive - The University of Nottingham/lab/report/Report 3/Umbrella (Yellow Line).png");

    // Load image
    Mat img = imread("C:/Users/autsj/OneDrive - The University of Nottingham/lab/report/Report 3/triangle small.png", IMREAD_COLOR);
    if (img.empty()) {
        cerr << "Error: Image not found or unable to load.\n";
        return -1;
    }

    // Convert image to grayscale
    Mat gray_img;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);

    // Variables to store maximum matches and corresponding template
    int maxMatches = 0;
    string bestTemplate;
    Mat bestImgMatches;

    // Loop through templates
    for (const auto& templatePath : templates) {
        // Load template
        Mat templ = imread(templatePath, IMREAD_COLOR);
        if (templ.empty()) {
            cerr << "Error: Template " << templatePath << " not found or unable to load.\n";
            continue;
        }

        // Perform feature matching
        featureMatching(templ, gray_img, templatePath, maxMatches, bestTemplate, bestImgMatches);
    }

    // Print the symbol with the highest number of good matches
    cout << "Symbol with the highest number of good matches: " << bestTemplate << endl;
    cout << "Number of good matches: " << maxMatches << endl;

    namedWindow("Best Matches", WINDOW_FULLSCREEN);
    // Display the image with the highest number of good matches
    imshow("Best Matches", bestImgMatches);

    waitKey(0);

    return 0;
}
