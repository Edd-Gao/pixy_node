#include <stdio.h>
#include <math.h>
#include "MOT.h"

#ifndef DRONE_LOC_H
#define DRONE_LOC_H

#define drone_screen_Len	1
#define drone_screen_Wid	2
#define drone_camera_Dist	3

struct cor_to_ball_s{
	double corP_x[4];
	double corP_y[4];
	double corP_z[4];
};

/*brief
	this function is to compute the coordinate of P in actual environment
	by inputting the three angles.
	it will return the address of struct
*/
extern int PointInThePhoto_PositionOfCamera(struct object_coordinate_s photo, struct cor_to_ball_s *solution);
extern int SetParameter(int parameter, int *value);
extern int GetParameter(int parameter, int *value);


#endif
