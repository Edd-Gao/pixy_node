#include "mot.h"

double object_x_cor[50]={0}, object_y_cor[50]={0};
int locationx=0,locationy=0;
int H_MIN = 139;
int H_MAX = 256;
int S_MIN = 141;
int S_MAX = 256;
int V_MIN = 103;
int V_MAX = 256;
//default capture width and height  
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame   
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area   
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
//const string windowName = "Original Image";
//const string windowName1 = "HSV Image";
//const string windowName2 = "Thresholded Image";
//const string windowName3 = "After Morphological Operations";
//const string trackbarWindowName = "Trackbars";

//The following for canny edge detec 
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";



int main(int argc, char* argv[])
{

  struct object_coordinate_s coordinate;
  Mat cameraFeed;

	//video capture object to acquire webcam feed?
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	waitKey(1000);

	while(1){
		//store image to matrix
		capture.read(cameraFeed);

		src = cameraFeed;

  		if( !src.data )
  		{ return -1; }

		//convert frame from BGR to HSV colorspace
    if (setHSVThreshold(Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX)) == 0){
      object_track(cameraFeed, &coordinate);
    }

		waitKey(30);
	}
	return 0;
}
