#include <stdio.h>
#include <math.h>
#include "MOT.h"

#ifndef DRONE_LOC_H
#define DRONE_LOC_H

struct cor_to_ball_s{
	double corP_x;
	double corP_y;
	double corP_z;
	double angle_coordinate;  //the angle between the two coordinate
};

/*brief
	this function is to compute the coordinate of P in actual environment
	by inputting the three angles.
	it will return the address of struct
*/
extern int PointInThePhoto_PositionOfCamera(struct object_coordinate_s photo, struct cor_to_ball_s *solution);
extern void Set_screen_Len(int *value);
extern void Set_screen_Wid(int *value);
extern void Set_camera_Dist(int *value);
extern void Set_distance_of_MO(int *value);
extern void Set_distance_of_LR(int *value);
extern void Get_screen_Len(int *value);
extern void Get_screen_Wid(int *value);
extern void Get_camera_Dist(int *value);
extern void Get_distance_of_MO(int *value);
extern void Get_distance_of_LR(int *value);


#endif
