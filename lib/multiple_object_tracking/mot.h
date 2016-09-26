#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Object.h"

struct object_coordinate_s{
 int l_coordinatex, l_coordinatey;
 int r_coordinatex, r_coordinatey;
 int s_coordinatex, s_coordinatey;
 int m_coordinatex, m_coordinatey;
};

int setHSVThreshold(Scalar min, Scalar max);

int setFrameHeightWidth(unsigned int height,unsigned int width);

int getHSVThreshold(Scalar * min, Scalar * max);

int getFrameHeightWidth(unsigned int * height,unsigned int * width);

int object_track(Mat cameraFeed, struct object_coordinate_s * coordinate );
