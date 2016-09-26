//#include <stdio.h>
#include "drone_loc.h"
#include "MOT.h"

#define pi 3.1416

int main()
{
	int i=0;
	int len_value = 128, wid_value = 64, dist_value = 100;
	struct cor_to_ball_s answer_temp;
	struct object_coordinate_s ball_temp;
	ball_temp.l_coordinate[0] = 36;
	ball_temp.l_coordinate[1] = 18;
	ball_temp.r_coordinate[0] = 49;
	ball_temp.r_coordinate[1] = 18;
	ball_temp.m_coordinate[0] = 41;
	ball_temp.m_coordinate[1] = 18;

	SetParameter(drone_screen_Len, &len_value);
	SetParameter(drone_screen_Wid, &wid_value);
	SetParameter(drone_camera_Dist, &dist_value);
	PointInThePhoto_PositionOfCamera(ball_temp, &answer_temp);

	for(i=0; i<4; i++)
	{
		printf("(%.4f,\t %.4f,\t %.4f)\n", answer_temp.corP_x[i], answer_temp.corP_y[i], answer_temp.corP_z[i]);
		//“.”前面那个参数是结构体，“->”前面的参数是结构体指针
	}
	return 0;
}

