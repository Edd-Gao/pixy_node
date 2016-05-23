#include "drone_loc.h"
#include "MOT.h"	//the library from Likun

static int screen_Len = 0;      //define the camera parameters
static int screen_Wid = 0;
static int camera_Dist= 0;
static int d_MO = 0;			//define the distance of balls
static double yR = 0.0;			//yR equals the half of d_LR. yR=d_LR/2


static const double pi = 3.1416;
static const int OK = 1;	//maybe defined in a head file.
static const int ERROR = -1;	//can be deleted later
static double xC, r, xB, yB;
static double sol = 0;	//store the result of 1_4 function
static double xP = 0, yP = 0;	//store the result of P in a certain coordinate

int solve_1_4(double angle_1, double angle_2, double angle_3);
void turn_coordinate(struct cor_to_ball_s * answer);
void substraction(double a[], double b[], double diff[]);
double dotproduct(double x[], double y[]);
double norm(double a[]);


void Set_screen_Len(int *value)
{
	screen_Len = *value;
}

void Set_screen_Wid(int *value)
{
	screen_Wid = *value;
}

void Set_camera_Dist(int *value)
{
	camera_Dist = *value;
}

void Set_distance_of_MO(int *value)
{
	d_MO = *value;
}

/* 	brief:
	    this function's name is setting the distance of LR, but LR is used for only once, yR is the useful parameter
	    so I just set yR's value and use yR
	    from the outside, you just need to input the distance of LR
	parameter:
	    value is the value of LR
*/
void Set_distance_of_LR(int *value)
{
	yR = (double)*value/2;	//turn int into double
}

void Get_screen_Len(int *value)
{
	*value = screen_Len;
}

void Get_screen_Wid(int *value)
{
	*value = screen_Wid;
}

void Get_camera_Dist(int *value)
{
	*value = camera_Dist;
}

void Get_distance_of_MO(int *value)
{
	*value = d_MO;
}

/* 	brief:
	    this function's name is setting the distance of LR, but LR is used for only once, yR is the useful parameter
	    so I just set yR's value and use yR

	    from the outside, you just need to input the distance of LR
	parameter:
	    value is the value of LR

*/
void Get_distance_of_LR(int *value)
{
	*value = (int)yR*2;	//turn double into int
}


int PointInThePhoto_PositionOfCamera(struct object_coordinate_s photo, struct cor_to_ball_s *solution)
{
    double camera_P[3]={screen_Len/2, screen_Wid/2, camera_Dist};         //the coordinates of P point
    double screen_Ri[3]={photo.r_coordinate[0],photo.r_coordinate[1],0};        //the coordinates of Ri point
    double screen_Li[3]={photo.l_coordinate[0],photo.l_coordinate[1],0};        //the coordinates of Li point
    double screen_Mi[3]={photo.m_coordinate[0],photo.m_coordinate[1],0};        //the coordinates of Mi point
    
    double vector_P_Ri[3];	//vector P->Ri
	double vector_P_Li[3];	//vector P->Li
	double vector_P_Mi[3];	//vector P->Mi
	double vector_P_A[3];	//A��P_Ri_Liƽ����һ�㣬����ֱ��A_Mi��ֱP_Ri_Liƽ��
	double n[3];
	double Za, Xa, Ya;
	double angle_a, angle_b, angle_r;
	int function_state = 0;

    substraction(camera_P, screen_Ri, vector_P_Ri);       //the coordinates of vector P_Ri
    substraction(camera_P, screen_Li, vector_P_Li);       //the coordinates of vector P_Li    
    substraction(camera_P, screen_Mi, vector_P_Mi);       //the coordinates of vector P_Mi
 
	n[0] = vector_P_Ri[1] * vector_P_Li[2] - vector_P_Ri[2] * vector_P_Li[1];
    n[1] = vector_P_Ri[2] * vector_P_Li[0] - vector_P_Ri[0] * vector_P_Li[2];
    n[2] = vector_P_Ri[0] * vector_P_Li[1] - vector_P_Ri[1] * vector_P_Li[0];         //the normal vector of plane P_Ri_Li
	Za = ( n[0]*n[2]*(screen_Len/2-photo.m_coordinate[0]) + n[1]*n[2]*(screen_Wid/2-photo.m_coordinate[1]) + n[2]*n[2]*camera_Dist ) / (n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
	Xa = n[0]/n[2]*Za + photo.m_coordinate[0];
	Ya = n[1]/n[2]*Za + photo.m_coordinate[1];
	//A as a point belong to plane P_Ri_Li, it makes vector A_Mi perpendicular to plane P_Ri_Li
	//vector_P_A[3] = {Xa-screen_Len/2, Ya-screen_Wid/2, Za-camera_Dist };			  //the coordinates of vector P_A
	vector_P_A[0] = Xa-screen_Len/2; vector_P_A[1] = Ya-screen_Wid/2; vector_P_A[2] = Za-camera_Dist;
	
	angle_a = acos( dotproduct(vector_P_Ri, vector_P_Li)/( norm(vector_P_Ri) * norm(vector_P_Li) ) );	   //calculate the value of angle a
	angle_b = acos( dotproduct(vector_P_Ri, vector_P_A)/( norm(vector_P_Ri) * norm(vector_P_A) ) );	  //calculate the value of angle b
	angle_r = asin( dotproduct(vector_P_Mi, n)/( norm(vector_P_Mi) * norm(n) ) );	  //calculate the value of angle r
	function_state = solve_1_4(angle_a, angle_b, angle_r);
	if(function_state == OK)
	{
		turn_coordinate( solution );
	}
	else
	{
		printf("cannot locate the drone\n");
	}
    return OK;
}


void substraction(double a[], double b[], double diff[])
{
	int i=0;
    for (i = 0; i < 3; i++) {
        diff[i] = b[i] - a[i];
    }
}                   //calculate vector coordinates

double dotproduct(double x[], double y[])
{
    double dot = 0;
	int i=0;
    for (i = 0; i < 3; i++) {
        dot += x[i] * y[i];
    }
    return dot;
}                  //calculate dot product

double norm(double a[])
{
    double sum = 0;
	int i = 0;
    for (i = 0; i<3; i++) {
        sum += pow(a[i], 2);
    }
    return sqrt(sum);
}                  //calculate vector norm


int solve_1_4(double angle_a, double angle_b, double angle_r)
{
	double	xC = yR/tan(angle_a);
	double	r = yR/sin(angle_a);	//the radium of the circle
	double	xB = xC - r*cos(2*angle_b-angle_a);
	double	yB = -r*sin(2*angle_b-angle_a);

	double	B2 = xB*xB + yB*yB;		//define some parameter help to compute
	double	n = tan(angle_r) * tan(angle_r);
	double	v = -2*xC*xB + B2;

	double c0 = -(d_MO*d_MO) * B2 + n*v*v;
	double c1 = (d_MO*d_MO) * xB*2 + 4*n*v*xC;
	double c2 = B2 - (d_MO*d_MO) - 2*n*v + 4*n*xC*xC;
	double c3 = -(4*n*xC + 2*xB);
	double c4 = n+1;

	double d0, d1, d2, delta;
	double k_BM;	//define the gradient
	double x1, x2;	//store the temp result of function

	double x,xx,f,k,temp_x;
	int i=0;
	x=15.0;

	for(i=0; i<10; i++)
	{
		temp_x = x;	//remain unchanged
		f = c4*x*x*x*x + c3*x*x*x + c2*x*x + c1*x +c0;	//the value of function
		k = 4*c4*x*x*x + 3*c3*x*x + 2*c2*x + c1;		//the value of pitch
		xx = x - f/k;
		while((fabs(x-xx))>(1*10e-5) && (fabs(x) < 15.1))
		{
			x=xx; 
			f = c4*x*x*x*x + c3*x*x*x + c2*x*x + c1*x +c0;	//the value of function
			k = 4*c4*x*x*x + 3*c3*x*x + 2*c2*x + c1;		//the value of pitch
			xx=x-f/k;
		}
		
		if(fabs(x) < 15.1)	//if jump out of the loop because of finding the right answer
		{
			printf("answer = %.4f\n",x);
			sol = xx;
			break;
		}
		x = temp_x - 3.0;	//start another loop
	}


	if(sol != 0)	//compute the 1_2 answer
	{
		printf("sol = %.4f\n",sol);
		k_BM = (-yB)/(sol-xB);
		d0 = xC*xC + (k_BM*sol)*(k_BM*sol) - r*r;
		d1 = -2*(xC + k_BM*k_BM*sol);
		d2 = k_BM*k_BM + 1;
		delta = d1*d1 - 4*d0*d2;
		x1 = (-d1 + sqrt(delta))/(2*d2);
		x2 = (-d1 - sqrt(delta))/(2*d2);
		if((x1-xB) < 10e-5)
			xP = x2;
		else
			xP = x1;
		printf("xP = %.4f\n",xP);
		yP = k_BM * (xP-sol);
		printf("yP = %.4f\n\n",yP);
		return OK;
	}
	else
		return ERROR;
}

void turn_coordinate(struct cor_to_ball_s *answer)
{

	double angle;

	angle = asin(sol/d_MO);
	answer->corP_z = xP * cos(angle);
	answer->corP_x = xP * sin(angle);
	answer->corP_y = yP;
	answer->angle_coordinate = angle;  //save the angle to struct
	printf("the angle between pixy and balls is %.2f", angle);		
}


