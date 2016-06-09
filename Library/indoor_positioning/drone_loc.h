#include <stdio.h>
#include <math.h>
#include "MOT.h"

#ifndef DRONE_LOC_H
#define DRONE_LOC_H

struct reference_coordinate_s{
	double x;
	double y;
	double z;
	double referenceAngle;  //the angle between the two coordinate
};

/*brief
	this function is to compute the coordinate of P in actual environment
	by inputting the three angles.
	it will return the address of struct
*/
extern int Image_To_Reference_Coordinate(struct image_coordinate_s imgCoordinate, struct reference_coordinate_s *refCoordinate);
extern void Set_Screen_Len(int *value);
extern void Set_Screen_Wid(int *value);
extern void Set_Camera_Distance(int *value);
extern void Set_Distance_of_MO(int *value);
extern void Set_Distance_of_LR(int *value);
extern int Get_Screen_Len();
extern int Get_Screen_Wid();
extern int Get_Camera_Dist();
extern int Get_Distance_of_MO();
extern int Get_Distance_of_LR();


#endif
