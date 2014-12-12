#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

#define RED CV_RGB(255,0,0)
#define GREEN CV_RGB(0,255,0)
#define BLUE CV_RGB(0,0,255)

// Plot Parameters
#define tick_size 3 // pixels
#define plot_width 200
#define plot_height 200
#define margin 20+plot_height/2.0
#define margin2 2*margin+plot_height/2.0
    
#define PI (3.141592653)
#define arrowMag 9
#define drawArrow( image, p, q,  color, thickness, line_type, shift) \
    line(image, p, q, color, thickness, line_type, shift);\
    Point r(p);\
    double angle = atan2((double)(p).y-(q).y, (double)(p).x-(q).x);\
    r.x = (int) ( (q).x +  arrowMag * cos(angle + PI/4));\
    r.y = (int) ( (q).y +  arrowMag * sin(angle + PI/4));\
    line(image, r, q, color, thickness, line_type, shift);\
    r.x = (int) ( (q).x +  arrowMag * cos(angle - PI/4));\
    r.y = (int) ( (q).y +  arrowMag * sin(angle - PI/4));\
    line(image, r, q, color, thickness, line_type, shift);

class BallTracker {

    private:
    VideoCapture kinect;
    
    // Pixel Frames
    Mat depth_map;
    Mat depth_img;
    Mat point_cld;
    Mat point_img;
    Mat rgb_img;
    Mat hsv_img;
    Mat thres_img;
    Mat thres2_img;
    Mat prev_thres_img;
    Mat valid_mask;
    
    // Stored Positions
    vector<Vec3f> circles, circles2, circles3;
    Mat pos3;
    Mat vel3;
    vector<Point> mes2;
    vector<Point> pos2;
    vector<Point> vel2;
    Vec3f position;
    Vec3f velocity;
        
    // Get Properties
    int width;
    int height;
    int fps;
    
    // Visual Ball Tracking        
    Point center;
    Point predicted;
    Point corrected;
    Point direction;
    
    int radius;
    string color;
    
    Scalar hsv_min;
    Scalar hsv_max;
    Scalar hsv_min2;
    Scalar hsv_max2;
    
    int frames_between_valid;
        
    // Filter Parameters
    int erodeAmt;
    int dilateAmt;
    int alpha;
	    
    KalmanFilter KF;
    Mat processNoise;
    Mat_<float> measurement; 

    KalmanFilter KF3;
    Mat processNoise3;
    Mat_<float> measurement3;
    
    public:
    BallTracker(void);
    BallTracker(string color);
    ~BallTracker(void);
    
    int run(void);
    Vec3f getPosition(void);
    Vec3f getVelocity(void);
    
};

