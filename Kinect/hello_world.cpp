#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

int main(){
    cout << "opening device(s)" << endl;

    VideoCapture sensor1;
    sensor1.open(CV_CAP_OPENNI);

    if( !sensor1.isOpened() ){
        cout << "Can not open capture object 1." << endl;
        return -1;
    }

    for(;;){
        Mat depth1;

        if( !sensor1.grab() ){
            cout << "Sensor1 can not grab images." << endl;
            return -1;
        }else if( sensor1.retrieve( depth1, CV_CAP_OPENNI_DEPTH_MAP ) ) imshow("depth1",depth1);

        if( waitKey( 30 ) == 27 )   break;//ESC to exit

   }
}


int main(){

    // Open connection to Kinect
    cout << "Connecting to Kinect" << endl;
    
    VideoCapture kinect;
    kinect.open(CV_CAP_OPENNI);
    
    if ( !kinect.isOpened() ){
        cout << "Can't connect to Kinect" << endl;
        return 1;
    }
    
    // Initialize variables
    
    int width  = kinect.get( CV_CAP_PROP_FRAME_WIDTH );
    int height = kinect.get( CV_CAP_PROP_FRAME_HEIGHT);
    cout << "FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;
    
    CvSize size = cvSize(height, width);
    IplImage* hsv_frame  = cvCreateImage(size, IPL_DEPTH_8U, 3);
    IplImage* threshold  = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage* threshold2 = cvCreateImage(size, IPL_DEPTH_8U, 1);
    
    // Give color thresholds to detect
    CvScalar hsv_min  = cvScalar(0, 50, 170, 0);
    CvScalar hsv_max  = cvScalar(10, 180, 256, 0);
    CvScalar hsv_min2 = cvScalar(170, 50, 170, 0);
    CvScalar hsv_max2 = cvScalar(256, 180, 256, 0);
    

    while( kinect.grab() ){
        
        Mat depth_map;
        Mat point_cld;
        Mat rgb_img;
        Mat hsv_img;
        temp = 
        
        kinect.retrieve( depth_map, CV_CAP_OPENNI_DEPTH_MAP );
        kinect.retrieve( point_cld, CV_CAP_OPENNI_POINT_CLOUD_MAP );
        kinect.retrieve( rgb_img, CV_CAP_OPENNI_BGR_IMAGE );
        
        
        // Convert to HSV Color Space
        cvCvtColor( rgb_img, hsv_img, CV_BGR2HSV );
        
        // Thresholding
        // Handle color wrap around by combining two halves
        cvInRange( hsv_img, hsv_min, hsv_max, thres_img );
        cvInRange( hsv_img, hsv_min2, hsv_max2, thres2_img );
        cvOr( thres_img, thres_img, thres2_img);
        
        // Hough Transform to detect circles
        // Works better with smoothing first
        cvSmooth( thres_img, thres_img, CV_GAUSSIAN, 9, 9 );
        cvSeq* circles = cvHoughCircles(thres_img, temp, CV_HOUGH_GRADIENT, 2, thres_img->height/4. 100. 40. 20. 200);
        
         for (int i = 0; i < circles->total; i++)
                    {
                        float* p = (float*)cvGetSeqElem( circles, i );
                        printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
                            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                             3, CV_RGB(0,255,0), -1, 8, 0 );
                            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                             cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
                     }
 
        
        // Display Image
        namedWindow( "Original Image", CV_WINDOW_AUTOSIZE );
        imshow( "Original Image", rgb_img );
        imshow( "HSV Image", hsv_img );
        imshow( "Thresholded Image", thres_img ); 
        imshow( "Tracked Image", track_img );
        imshow( "Depth Map", depth_map );
        
        // hit ESC to exit
        if ( waitKey(30) == 27 ) break;
    }
    cout << "Can't grab images from Kinect" << endl;

}
