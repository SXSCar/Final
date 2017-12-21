#include <cstdlib>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "GPIOlib.h"

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
//#define _DEBUG

using namespace cv;
using namespace std;
using namespace GPIO;

const string CAM_PATH = "/dev/video0";
const string MAIN_WINDOW_NAME = "Processed Image";
const string CANNY_WINDOW_NAME = "Canny";

const int CANNY_LOWER_BOUND = 50;
const int CANNY_UPPER_BOUND = 150;
const int HOUGH_THRESHOLD   = 40;
int lineCount = 0;
int isVisible = 0;
int watch = 41;
int shouldIdle = 0;
int currentStage = 1;
int speed = 5;
int turnWatch = 0;
int turnWatchFlag = 0;

int main() {
    GPIO::init();

    turnTo(0);

    VideoCapture capture(CAM_PATH);
    //If this fails, try to open as a video camera, through the use of an integer param
    if (!capture.isOpened()) {
        capture.open(atoi(CAM_PATH.c_str()));
    }

    double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);            //the width of frames of the video
    double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);        //the height of frames of the video
    clog << "Frame Size: " << dWidth << "x" << dHeight << endl;

    Mat image;
    int i = 1;
    //投票决定应该turn哪边
    int should_turn_left = 0;
    while (true) {

        if (turnWatchFlag == 1) {
            turnWatch++;

            if (turnWatch == 4) {
                turnWatch = 0;
                turnWatchFlag = 0;
                clog << "Recover TurnTo" << endl;

                if (shouldIdle == 0) {
                    turnTo(0);
                }
            }
        }

        if (lineCount == 4 && currentStage == 1) {

            clog << "Stage 1 Finished" << endl;

            shouldIdle++;

            currentStage++;

            lineCount = 0;
        } else if (lineCount == 9 && currentStage == 2) {

            clog << "Stage 2 Finished" << endl;

            shouldIdle++;

            currentStage++;

            lineCount = 0;
        } else if (lineCount == 5 && currentStage == 3) {

            clog << "Stage 3 Finished" << endl;

            shouldIdle++;

            currentStage++;
        }

        watch++;
        controlLeft(1, speed);
        controlRight(1, speed);

        if (shouldIdle == 0) {
            if (should_turn_left > 0) {
                clog << "turn left" << endl;

                turnWatchFlag = 1;

                turnWatch = 0;

                turnTo(-35);
            }
            if (should_turn_left < 0) {
                clog << "turn right" << endl;

                turnWatchFlag = 1;

                turnWatch = 0;

                turnTo(10);
            }
            i = 0;
        } else if (shouldIdle > 0) {
            shouldIdle++;

            // 3.95V: interval1 : 20
            //        interval2 : 77
            //        interval3 : 20

            // 4.08V: interval1 : 25
            //        interval2 : 73
            //        interval3 : 20

            int interval1 = 33;

            if (currentStage == 2) {
                interval1 += 5;
            }

            // delay to turn left first corner
            if (shouldIdle == interval1) {
                if (currentStage == 4) {
                    stopLeft();
                    stopRight();

                    return 0;
                } else {
                    turnTo(-32);
                }
            }

            int interval2 = interval1 + 58;

            // how long to turn left
            if (shouldIdle == interval2) {
                turnTo(0);

                clog << "Recover!!!!!!!!!!!" << endl;
            }

            // delay to turn left second corner
            int interval3 = interval2;

            if (shouldIdle == interval3) {
                shouldIdle = 0;

                lineCount = 0;

                i = 0;
            }
        }
        should_turn_left = 0;
        i++;
        capture >> image;
        if (image.empty())
            break;

        Rect roi(0, image.rows / 3, image.cols, image.rows / 2);
        Mat imgROI = image(roi);

        //Canny algorithm
        Mat contours;
        //blur(imgROI,imgROI,Size(30,30));
        Canny(imgROI, contours, CANNY_LOWER_BOUND, CANNY_UPPER_BOUND);

#ifdef _DEBUG
        imshow(CANNY_WINDOW_NAME, contours);
#endif

        vector<Vec4i> lines;
        HoughLinesP(contours, lines, 1, PI / 180, HOUGH_THRESHOLD, 10, 5);
        Mat result(imgROI.size(), CV_8U, Scalar(255));
        imgROI.copyTo(result);
        clog << lines.size() << endl;

        //Draw the lines and judge the slope
        int color = 50;
        int flag = 0;
        for (vector<Vec4i>::const_iterator it = lines.begin(); it != lines.end(); ++it) {
            line(result, Point((*it)[0], (*it)[1]), Point((*it)[2], (*it)[3]), Scalar(color, color, 255), 20, CV_AA);
            float slope = ((float) (*it)[3] - (*it)[1]) / ((*it)[2] - (*it)[0]);
            //clog << "slope" << slope << endl;
            if (slope >= -0.4 && slope <= 0.4) {
                if (isVisible == 0) {
                    isVisible = 1;
                }
                flag = 1;
            }
            if (slope <= 2 && slope >= 0.4) {
                should_turn_left++;
            }
            if (slope >= -2 && slope <= -0.4) {
                should_turn_left--;
            }
            color += 50;
            color %= 256;
            //Filter to remove vertical and horizontal lines,
            //and atan(0.09) equals about 5 degrees.
        }

        if (flag == 0 && isVisible == 1 && watch > 40) {
            clog << "count++ " << lineCount << endl;
            lineCount++;
            isVisible = 0;
            watch = 0;
        }

#ifdef _DEBUG
        stringstream overlayedText;
        overlayedText<<"Lines: "<<lines.size();
        putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
        imshow(MAIN_WINDOW_NAME,result);
#endif

        lines.clear();
        waitKey(1);
    }
    return 0;
}
