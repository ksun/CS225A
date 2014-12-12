#include "ball_tracking.hpp"

BallTracker::BallTracker(string c = "Green"){
    // Open connection to Kinect
    cout << "Connecting to Kinect" << endl;
    
    kinect.open(CV_CAP_OPENNI);

    if ( !kinect.isOpened() ){
        cout << "Can't connect to Kinect" << endl;
    }
    
    
    // Registration - Loads calibration data to align depth map with visual camera
    if( kinect.get( CV_CAP_PROP_OPENNI_REGISTRATION ) == 0 ) 
        kinect.set( CV_CAP_PROP_OPENNI_REGISTRATION, 1 );

    // Get Properties
    width  = kinect.get( CV_CAP_PROP_FRAME_WIDTH );
    height = kinect.get( CV_CAP_PROP_FRAME_HEIGHT);
    fps = kinect.get( CV_CAP_PROP_FPS );
    cout << "Resolution    " << width << "x" << height << "    FSP    " << fps << endl;
    
    color = c;
    // Give ball color thresholds to detect
 
    if (color == "Pink"){
     hsv_min  = Scalar(0, 50, 170, 0);
     hsv_max  = Scalar(10, 180, 256, 0);
     hsv_min2 = Scalar(170, 50, 170, 0);
     hsv_max2 = Scalar(256, 180, 256, 0);
    } else {
        if (color == "Green") {
     hsv_min  = Scalar(40, 50, 70, 0);
     hsv_max  = Scalar(80, 256, 256, 0);
    } else if (color == "Yellow") {
     hsv_min  = Scalar(20, 30, 50, 0);
     hsv_max  = Scalar(60, 256, 256, 0);
    } else if (color == "Purple") {
     hsv_min  = Scalar(115, 30, 50, 0);
     hsv_max  = Scalar(160, 256, 256, 0);
    } else {
    cout << "Color not supported: " << c << endl;
    }
    
     hsv_min2 = hsv_min;
     hsv_max2 = hsv_max;
    
    cout << "Detecting " << color << " Ball" << endl;
    }
    
    prev_thres_img = Mat::zeros(height, width, CV_8UC1);
    frames_between_valid = 1;
    pos3.push_back(Vec3f(0,0,0));
    vel3.push_back(Vec3f(0,0,0));
        
    // Filter Parameters
    erodeAmt = 1;
    dilateAmt = 1;
    alpha = 0.9;
    
    // Kalman Filter Computer Vision Tracking
    /*
    KalmanFilter KF3(6, 3, 0);
    KF3.transitionMatrix = *(Mat_<float>(6,6) << 1,0,0,1,0,0,
                                                0,1,0,0,1,0,
                                                0,0,1,0,0,1,
                                                0,0,0,1,0,0,
                                                0,0,0,0,1,0,
                                                0,0,0,0,0,1 );

	Mat processNoise(6, 1, CV_32F);
	Mat measurement = Mat::zeros(3, 1, CV_32F); */
	KF.init(4,2,0);
	processNoise.create(4, 1, CV_32F);
	measurement = Mat::zeros(2, 1, CV_32F); 

	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);
    // Too slow, change variables
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));
    setIdentity(KF.errorCovPost, Scalar::all(.1));

}

BallTracker::~BallTracker(void){
}

int BallTracker::run(void) {

    for (;;){
        
        if ( !kinect.grab() ) {
            cout << "Cannot grab images. Check Kinect Connection." << endl;
            return -1;
        }
        
        if (!( kinect.retrieve( depth_map, CV_CAP_OPENNI_DEPTH_MAP ) &&
            kinect.retrieve( point_cld, CV_CAP_OPENNI_POINT_CLOUD_MAP ) &&
            kinect.retrieve( rgb_img, CV_CAP_OPENNI_BGR_IMAGE ) &&
            kinect.retrieve( valid_mask, CV_CAP_OPENNI_VALID_DEPTH_MASK ))) {
            cout << "Could not retrieve image. Check Kinect." << endl;
            return -1;
        }

        // Scale depth map to show as image
        //depth_map.convertTo( depth_img, CV_8UC1, 255.0/8192.0 ); //scale 16 bit to 8 bit

        // Pixel Filtering of depth map / point cloud
        // Get rid of holes in depth map - necessary? median filter instead?
        point_img = point_cld.clone();
        dilate(point_img,point_img,Mat(),Point(-1,-1),dilateAmt);
        medianBlur(point_img, point_img, 3);
        erode(point_img,point_img,Mat(),Point(-1,-1),erodeAmt);
        
        // Weighted Moving Average
        // Smooth depth map/ remove jitter and noise
        // Joint Bilateral Filter - too slow? unnecessary?
        
        // Convert RGB to HSV Color Space
        cvtColor( rgb_img, hsv_img, CV_BGR2HSV );
        
        // Thresholding
        // Handle color wrap around by combining two halves (for red)
        inRange( hsv_img, hsv_min, hsv_max, thres_img );
        inRange( hsv_img, hsv_min2, hsv_max2, thres2_img );
        thres_img = thres_img | thres2_img;
        
        //namedWindow( "Thresholded Pre Image", CV_WINDOW_AUTOSIZE );
        //imshow( "Thresholded Pre Image", thres_img );

        // Get rid of more noise via Erode/Dialate
		erode(thres_img,thres_img,Mat(),Point(-1,-1),erodeAmt);
		dilate(thres_img,thres_img,Mat(),Point(-1,-1),dilateAmt);
		
		// Fill Holes - remove - induces white flickering
		//thres2_img = Scalar::all(255) - thres_img;
		//floodFill(thres2_img, Point(0,0), Scalar(0));
		//thres_img += thres2_img;
		
        // Hough Transform to detect circles
        // Works better with smoothing first
        GaussianBlur( thres_img, thres_img, Size(9, 9), 2, 2 );

        // weight then blur or blur then weight?
        // Weighted Average to Reduce Flickering
        addWeighted( thres_img, alpha, prev_thres_img, 1.0-alpha, 0.0, thres_img );
        prev_thres_img = thres_img;
        
        // TODO Use ROI for specified search space
        // res, min_dist, canny_upper, center_detection, min_r, max_r
        HoughCircles(thres_img, circles, CV_HOUGH_GRADIENT, 2, height, 400, 40, 5, 21);
        
        // Draw Trajectory
        for ( int i = 0; i < ((int) mes2.size())-1; ++i ) { 
            line( rgb_img, mes2[i], mes2[i+1], BLUE, 1 );
            line( rgb_img, pos2[i], pos2[i+1], RED, 1 );
        }
        
        // Draw Circles
        //TODO change to multiple circles and select one closest to last
        // Should only have at most one circle
        for (vector<Vec3f>::iterator it = circles.begin(); it != circles.end(); ++it ){ 
            //printf("Ball! x=%f y=%f r=%f\n\r",(*it)[0],(*it)[1],(*it)[2] );
            center = Point(round((*it)[0]), round((*it)[1]));
            radius = round((*it)[2]);
            
            //cout<<depth_map.at<unsigned short>(int(center.y), int(center.x))<<endl;
        	
            // Kalman Filter on 2D point from Computer Vision Tracking
            // Get rid of large jumps in tracking
            Mat prediction = KF.predict();
            predicted = Point(prediction.at<float>(0), prediction.at<float>(1));
            measurement.at<float>(0) = center.x;
            measurement.at<float>(1) = center.y;
            Mat correction = KF.correct(measurement);
            corrected = Point(correction.at<float>(0), correction.at<float>(1));

            if (pos2.empty()) {
                direction = Point(0,0);
            } else {
                direction = corrected - pos2.back();
            }
            vel2.push_back(direction);
            mes2.push_back(center);
            pos2.push_back(corrected);

            // Retrieve 3D position data from depth map/ point cloud
            // Calculate 3D velocity vector from two frames
            // TODO put velocity vector through Kalman Filter for stability or return a weighted average
            // Take median of all within range?
            //Mat roi = point_cld( Rect(center.x-radius, center.y-radius, 2*radius, 2*radius));
            //medianBlur(roi, roi, Size(3,3));
            position = point_img.at<Vec3f>(int(center.y), int(center.x));
            velocity = Vec3f(0,0,0);
            if (position != Vec3f(0,0,0)) { // Valid Point
                velocity = (position - pos3.at<Vec3f>(pos3.total()-1)) * (1.0 / (frames_between_valid * fps));
                vel3.push_back(velocity);
                pos3.push_back(position);
                frames_between_valid = 1;
            } else {
                frames_between_valid++;
            }
            if (frames_between_valid > 1000) {
                cout << "No depth data for ball. Please move backwards." << endl;
            }
            
            //cout << position << endl;
            //cout << velocity << endl;
            
            // Draw tracked circles
            circle( rgb_img, center, 3, BLUE, -1, 8, 0 );
            circle( rgb_img, center, radius, BLUE, 3, 8, 0 );
            circle( point_img, center, 3, BLUE, -1, 8, 0 );
            circle( point_img, center, radius, BLUE, 3, 8, 0 );
            circle( thres_img, center, 3, BLUE, -1, 8, 0 );
            circle( thres_img, center, radius, BLUE, 3, 8, 0 );
            circle( point_img, corrected, 3, RED, -1, 8, 0 );
            circle( point_img, corrected, radius, RED, 3, 8, 0 );

            // Draw predicted, observed, corrected, and velocity vectors
            drawArrow( rgb_img, corrected, corrected+(direction*2), RED, 2, CV_AA, 0 );
            //circle( rgb_img, center, 3, BLUE, -1, 8, 0 );
            circle( rgb_img, predicted, 3, GREEN, -1, 8, 0 );
            //circle( rgb_img, corrected, 3, RED, -1, 8, 0 );
        }

        // Plot 3d Trajectory
        Mat pos3_channels[3];
        Mat vel3_channels[3];
        split(pos3, pos3_channels);
        split(vel3, vel3_channels);

        Mat graph = Mat::zeros((plot_height+20)*2, plot_width*tick_size, CV_8UC3);
        // Title
        putText(graph, "Position (m)" , Point(20,20), FONT_HERSHEY_SIMPLEX, 0.5, RED, 1, CV_AA);
        putText(graph, "Velocity (cm/s)" , Point(20,round(graph.rows/2)), FONT_HERSHEY_SIMPLEX, 0.5, RED, 1, CV_AA);
        // Legend
        putText(graph, "x" , Point(10,2*plot_height), FONT_HERSHEY_SIMPLEX, 0.25, RED, 1, CV_AA);
        putText(graph, "y" , Point(10,2*plot_height+10), FONT_HERSHEY_SIMPLEX, 0.25, BLUE, 1, CV_AA);
        putText(graph, "z" , Point(10,2*plot_height+20), FONT_HERSHEY_SIMPLEX, 0.25, GREEN, 1, CV_AA);
        Point a, b;
        int start = (int)pos3.total() > plot_width ? (int)pos3.total() - plot_width : 0;
        for ( int t = start; t < ((int)pos3.total())-1; ++t ){
            // Position
            a = Point(tick_size*(t-start), round(pos3_channels[0].at<float>(t)*plot_height/2.0+margin));
            b = Point(tick_size*(t-start+1), round(pos3_channels[0].at<float>(t+1)*plot_height/2.0+margin));
            line( graph, a, b, RED, 1 ); //x
            a = Point(tick_size*(t-start), round(pos3_channels[1].at<float>(t)*plot_height/2.0+margin));
            b = Point(tick_size*(t-start+1), round(pos3_channels[1].at<float>(t+1)*plot_height/2.0+margin));
            line( graph, a, b, GREEN, 1 ); //z
            a = Point(tick_size*(t-start), round(pos3_channels[2].at<float>(t)*plot_height/2.0+margin));
            b = Point(tick_size*(t-start+1), round(pos3_channels[2].at<float>(t+1)*plot_height/2.0+margin));
            line( graph, a, b, BLUE, 1 ); //y
            
            // Velocity
            a = Point(tick_size*(t-start), round(vel3_channels[0].at<float>(t)*50*plot_height+margin2));
            b = Point(tick_size*(t-start+1), round(vel3_channels[0].at<float>(t+1)*50*plot_height+margin2));
            line( graph, a, b, RED, 1 ); //x
            a = Point(tick_size*(t-start), round(vel3_channels[1].at<float>(t)*50*plot_height+margin2));
            b = Point(tick_size*(t-start+1), round(vel3_channels[1].at<float>(t+1)*50*plot_height+margin2));
            line( graph, a, b, GREEN, 1 ); //z
            a = Point(tick_size*(t-start), round(vel3_channels[2].at<float>(t)*50*plot_height+margin2));
            b = Point(tick_size*(t-start+1), round(vel3_channels[2].at<float>(t+1)*50*plot_height+margin2));
            line( graph, a, b, BLUE, 1 ); //y
        }
       
        namedWindow( "Graph", CV_WINDOW_AUTOSIZE );
        imshow( "Graph", graph );
        
        // Display Image
        namedWindow( "Original Image", CV_WINDOW_AUTOSIZE );
        imshow( "Original Image", rgb_img );
        //namedWindow( "HSV Image", CV_WINDOW_AUTOSIZE );
        //imshow( "HSV Image", hsv_img );
        namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
        imshow( "Thresholded Image", thres_img );
        //namedWindow( "Depth Map", CV_WINDOW_AUTOSIZE ); 
        //imshow( "Depth Map", point_img );
        namedWindow( "Point Cloud", CV_WINDOW_AUTOSIZE ); 
        imshow( "Point Cloud", point_img);
        
        char key = waitKey(30);
        // hit ESC to exit
        if ( key == 27 ) break;
        // hit s to save
        else if( key == 's' ) {
           imwrite("_rgb.png", rgb_img);
           imwrite("_thres.png", thres_img);
           imwrite("_depth.png", depth_img);
           imwrite("_point.png", point_img);
           imwrite("_graph.png", graph);
        } 
        // hit c to clear
        else if( key == 'c' ) {
            mes2.clear();
            pos2.clear();
            vel2.clear();
            pos3 = Mat::zeros(1, 1, CV_32FC3);
            vel3 = Mat::zeros(1, 1, CV_32FC3);
        }
    }
    cout << "Can't grab images from Kinect" << endl;
    return -1;
}
    
Vec3f BallTracker::getPosition(void) {
    return  position; //z, x, y
}

Vec3f BallTracker::getVelocity(void) {
    return  velocity;
}
