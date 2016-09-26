#include "mot.h"
#include "Object.h"
#include<math.h>

//default capture width and height 
static int FRAME_WIDTH = 640;
static int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame   
static int MAX_NUM_OBJECTS=50;
//minimum and maximum object area  
static int MIN_OBJECT_AREA = 20*20;
static int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

static int locationx[4]={0,0,0,0},locationy[4]={0,0,0,0};
/*static int coordinate->l_coordinatex=0, coordinate->l_coordinatey=0;
static int coordinate->r_coordinatex=0, coordinate->r_coordinatey=0;
static int coordinate->s_coordinatex=0, coordinate->s_coordinatey=0;
static int coordinate->m_coordinatex=0, coordinate->m_coordinatey=0;*/
/*static int lx=0;ly=0;
static int rx=0,ry=0;
static int sx=0,sy=0;
static int mx=0,my=0;*/
static int t=0;
static int k=0;

static Scalar HSV_MIN(0,0,0),HSV_MAX(255,255,255);
/*object_coordinate_s coordinate_get;
object_coordinate_s *coordinate=&coordinate_get;*/

static void morphOps(Mat &thresh){

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));

	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

static int trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed, struct object_coordinate_s * coordinate)
{
	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
//
//	int i=0;
	bool objectFound = false;

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA)
				{
					Object object;
					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);
					objects.push_back(object);
					objectFound = true;
					printf("found");
				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true && objects.size()==4)
			{
				//draw object location on screen
				//drawObject(objects,cameraFeed);
				
				for(unsigned int i = 0;i < objects.size();i++)
				{
					locationx[i] = objects[i].getXPos();
					locationy[i] = objects[i].getYPos();
					//printf("X1 %d,Y1 %d\n",locationx[0],locationy[0]);
					//printf("X2 %d,Y2 %d\n",locationx[1],locationy[1]);
					//printf("X3 %d,Y3 %d\n",locationx[2],locationy[2]);
					//printf("X4 %d,Y4 %d\n",locationx[3],locationy[3]);					
				}		
						if(abs((locationy[2]-locationy[1])/(locationx[2]-locationx[1])-
								 (locationy[1]-locationy[0])/(locationx[1]-locationx[0]))<0.2)
						{
							if(locationx[0]>locationx[1])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[1];locationy[0]=locationy[1];
								locationx[1]=t;locationy[1]=k;
							}
							if(locationx[0]>locationx[2])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[2];locationy[0]=locationy[2];
								locationx[2]=t;locationy[2]=k;
							}
							if(locationx[1]>locationx[2])
							{
								t=locationx[1];k=locationy[1];
								locationx[1]=locationx[2];locationy[1]=locationy[2];
								locationx[2]=t;locationy[2]=k;
							}
							coordinate->l_coordinatex=locationx[0];coordinate->l_coordinatey=locationy[0];
							coordinate->r_coordinatex=locationx[1];coordinate->r_coordinatey=locationy[1];
							coordinate->s_coordinatex=locationx[2];coordinate->s_coordinatey=locationy[2];coordinate->m_coordinatex=locationx[3];coordinate->m_coordinatey=locationy[3];
					printf("coordinate->l_coordinatex %d,coordinate->l_coordinatey %d\n",locationx[0],locationy[0]);
					printf("coordinate->r_coordinatex %d,coordinate->r_coordinatey %d\n",locationx[1],locationy[1]);
					printf("coordinate->s_coordinatex %d,coordinate->s_coordinatey %d\n",locationx[2],locationy[2]);
					printf("coordinate->m_coordinatex %d,coordinate->m_coordinatey %d\n",locationx[3],locationy[3]);	
						}
						if(abs((locationy[3]-locationy[2])/(locationx[3]-locationx[2])-
								 (locationy[2]-locationy[1])/(locationx[2]-locationx[1]))<0.2)
						{
							if(locationx[1]>locationx[2])
							{
								t=locationx[1];k=locationy[1];
								locationx[1]=locationx[2];locationy[1]=locationy[2];
								locationx[2]=t;locationy[2]=k;
							}
							if(locationx[1]>locationx[3])
							{
								t=locationx[1];k=locationy[1];
								locationx[1]=locationx[3];locationy[1]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							if(locationx[2]>locationx[3])
							{
								t=locationx[2];k=locationy[2];
								locationx[2]=locationx[3];locationy[2]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							coordinate->l_coordinatex=locationx[1];coordinate->l_coordinatey=locationy[1];
							coordinate->r_coordinatex=locationx[2];coordinate->r_coordinatey=locationy[2];
							coordinate->s_coordinatex=locationx[3];coordinate->s_coordinatey=locationy[3];coordinate->m_coordinatex=locationx[0];coordinate->m_coordinatey=locationy[0];
					printf("coordinate->l_coordinatex %d,coordinate->l_coordinatey %d\n",locationx[1],locationy[1]);
					printf("coordinate->r_coordinatex %d,coordinate->r_coordinatey %d\n",locationx[2],locationy[2]);
					printf("coordinate->s_coordinatex %d,coordinate->s_coordinatey %d\n",locationx[3],locationy[3]);
					printf("coordinate->m_coordinatex %d,coordinate->m_coordinatey %d\n",locationx[0],locationy[0]);	
						}
						if(abs((locationy[3]-locationy[1])/(locationx[3]-locationx[1])-
								 (locationy[1]-locationy[0])/(locationx[1]-locationx[0]))<0.2)
						{
							if(locationx[0]>locationx[1])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[1];locationy[0]=locationy[1];
								locationx[1]=t;locationy[1]=k;
							}
							if(locationx[0]>locationx[3])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[3];locationy[0]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							if(locationx[1]>locationx[3])
							{
								t=locationx[1];k=locationy[1];
								locationx[1]=locationx[3];locationy[1]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							coordinate->l_coordinatex=locationx[0];coordinate->l_coordinatey=locationy[0];
							coordinate->r_coordinatex=locationx[1];coordinate->r_coordinatey=locationy[1];
							coordinate->s_coordinatex=locationx[3];coordinate->s_coordinatey=locationy[3];coordinate->m_coordinatex=locationx[2];coordinate->m_coordinatey=locationy[2];
					printf("coordinate->l_coordinatex %d,coordinate->l_coordinatey %d\n",locationx[0],locationy[0]);
					printf("coordinate->r_coordinatex %d,coordinate->r_coordinatey %d\n",locationx[1],locationy[1]);
					printf("coordinate->s_coordinatex %d,coordinate->s_coordinatey %d\n",locationx[3],locationy[3]);
					printf("coordinate->m_coordinatex %d,coordinate->m_coordinatey %d\n",locationx[2],locationy[2]);	
						}
						if(abs((locationy[3]-locationy[2])/(locationx[3]-locationx[2])-
								 (locationy[2]-locationy[0])/(locationx[2]-locationx[0]))<0.2)
						{
							if(locationx[0]>locationx[2])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[2];locationy[0]=locationy[2];
								locationx[2]=t;locationy[2]=k;
							}
							if(locationx[0]>locationx[3])
							{
								t=locationx[0];k=locationy[0];
								locationx[0]=locationx[3];locationy[0]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							if(locationx[2]>locationx[3])
							{
								t=locationx[2];k=locationy[2];
								locationx[2]=locationx[3];locationy[2]=locationy[3];
								locationx[3]=t;locationy[3]=k;
							}
							coordinate->l_coordinatex=locationx[0];coordinate->l_coordinatey=locationy[0];
							coordinate->r_coordinatex=locationx[2];coordinate->r_coordinatey=locationy[2];
							coordinate->s_coordinatex=locationx[3];coordinate->s_coordinatey=locationy[3];coordinate->m_coordinatex=locationx[1];coordinate->m_coordinatey=locationy[1];
					printf("coordinate->l_coordinatex %d,coordinate->l_coordinatey %d\n",locationx[0],locationy[0]);
					printf("coordinate->r_coordinatex %d,coordinate->r_coordinatey %d\n",locationx[2],locationy[2]);
					printf("coordinate->s_coordinatex %d,coordinate->s_coordinatey %d\n",locationx[3],locationy[3]);
					printf("coordinate->m_coordinatex %d,coordinate->m_coordinatey %d\n",locationx[1],locationy[1]);	
						}			
       return 0;
		}
		else //putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
      return -1;
	}
}
}

int setHSVThreshold(Scalar min, Scalar max){
  HSV_MIN = min;
  HSV_MAX = max;
  return 0;
}

int getHSVThreshold(Scalar * min, Scalar * max){
	*min = HSV_MIN;
	*max = HSV_MAX;
	return 0;
}

int setFrameHeightWidth(unsigned int height,unsigned int width){
	FRAME_WIDTH = width;
	FRAME_HEIGHT = height;
	return 0;
}

int getFrameHeightWidth(unsigned int * height,unsigned int * width){
	*width = FRAME_WIDTH;
	*height = FRAME_HEIGHT;
	return 0;
}


int object_track(Mat cameraFeed, struct object_coordinate_s * coordinate ){

  Mat threshold;
  Mat HSV;

  cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
  inRange(HSV,HSV_MIN,HSV_MAX,threshold);//需要修改成预设的值 check
  morphOps(threshold);

  vector <Object> objects;
  return trackFilteredObject(threshold,HSV,cameraFeed, coordinate);//

}
