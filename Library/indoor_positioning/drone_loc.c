#include "drone_loc.h"

static int screenLen = 0;      //define the camera parameters
static int screenWid = 0;
static int cameraDist= 0;
static int dMo = 0;			//define the distance of balls
static double yR = 0.0;			//yR equals the half of d_LR. yR=d_LR/2


static const double pi = 3.1416;

//maybe defined in a head file, can be deleted later
static const int OK = 1;
static const int ERROR = -1;

static double xC, r, xB, yB;
static double sol = 0;	//store the result of 1_4 function
static double xP = 0, yP = 0;	//store the result of P in a certain coordinate

int Solve_1_4(double angleA, double angleB, double angleR);
void Turn_Coordinate(struct reference_coordinate_s *answer);
void Substraction(double *a, double *b, double *diff);
double Dotproduct(double *x, double *y);
double Norm(double *a);


void Set_Screen_Len(int *value)
{
	screenLen = *value;
}

void Set_Screen_Wid(int *value)
{
	screenWid = *value;
}

//camera distance is the distance between sensor level and the lens level
void Set_Camera_Distance(int *value)
{
	cameraDist = *value;
}

void Set_Distance_of_MO(int *value)
{
	dMo = *value;
}

/* 	brief:
	    this function's name is setting the distance of LR, but LR is used for only once, yR is the useful parameter
	    so I just set yR's value and use yR
	    from the outside, you just need to input the distance of LR
	parameter:
	    value is the value of LR
*/
void Set_Distance_of_LR(int *value)
{
	yR = (double)*value/2;	//turn int into double
}

int Get_Screen_Len()
{
	return screenLen;
}

int Get_Screen_Wid()
{
	return screenWid;
}

int Get_Camera_Dist()
{
	return cameraDist;
}

int Get_Distance_of_MO()
{
	return dMo;
}

/* 	brief:
	    this function's name is setting the distance of LR, but LR is used for only once, yR is the useful parameter
	    so I just set yR's value and use yR

	    from the outside, you just need to input the distance of LR
	parameter:
	    value is the value of LR

*/
int Get_Distance_of_LR()
{
	return (int)yR*2;	//turn double into int
}


int Image_To_Reference_Coordinate(struct image_coordinate_s imgCoordinate, struct reference_coordinate_s *refCoordinate)
{
    double camera_P[3]={screenLen/2, screenWid/2, cameraDist};         //the coordinates of P point
    double screen_Ri[3]={imgCoordinate.r_coordinate[0],imgCoordinate.r_coordinate[1],0};        //the coordinates of Ri point
    double screen_Li[3]={imgCoordinate.l_coordinate[0],imgCoordinate.l_coordinate[1],0};        //the coordinates of Li point
    double screen_Mi[3]={imgCoordinate.m_coordinate[0],imgCoordinate.m_coordinate[1],0};        //the coordinates of Mi point
    
    double vector_P_Ri[3];	//vector P->Ri
	double vector_P_Li[3];	//vector P->Li
	double vector_P_Mi[3];	//vector P->Mi
	double vector_P_A[3];	//A��P_Ri_Liƽ����һ�㣬����ֱ��A_Mi��ֱP_Ri_Liƽ��
	double n[3];
	double Za, Xa, Ya;
	double angle_a, angle_b, angle_r;
	int function_state = 0;

	Substraction(camera_P, screen_Ri, vector_P_Ri);       //the coordinates of vector P_Ri
	Substraction(camera_P, screen_Li, vector_P_Li);       //the coordinates of vector P_Li
	Substraction(camera_P, screen_Mi, vector_P_Mi);       //the coordinates of vector P_Mi
 
	n[0] = vector_P_Ri[1] * vector_P_Li[2] - vector_P_Ri[2] * vector_P_Li[1];
    n[1] = vector_P_Ri[2] * vector_P_Li[0] - vector_P_Ri[0] * vector_P_Li[2];
    n[2] = vector_P_Ri[0] * vector_P_Li[1] - vector_P_Ri[1] * vector_P_Li[0];         //the normal vector of plane P_Ri_Li
	Za = ( n[0]*n[2]*(screenLen/2-imgCoordinate.m_coordinate[0]) + n[1]*n[2]*(screenWid/2-imgCoordinate.m_coordinate[1]) + n[2]*n[2]*cameraDist ) / (n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
	Xa = n[0]/n[2]*Za + imgCoordinate.m_coordinate[0];
	Ya = n[1]/n[2]*Za + imgCoordinate.m_coordinate[1];
	//A as a point belong to plane P_Ri_Li, it makes vector A_Mi perpendicular to plane P_Ri_Li
	//vector_P_A[3] = {Xa-screenLen/2, Ya-screenWid/2, Za-cameraDist };			  //the coordinates of vector P_A
	vector_P_A[0] = Xa-screenLen/2; vector_P_A[1] = Ya-screenWid/2; vector_P_A[2] = Za-cameraDist;
	
	angle_a = acos(Dotproduct(vector_P_Ri, vector_P_Li)/(Norm(vector_P_Ri) * Norm(vector_P_Li) ) );	   //calculate the value of angle a
	angle_b = acos(Dotproduct(vector_P_Ri, vector_P_A)/(Norm(vector_P_Ri) * Norm(vector_P_A) ) );	  //calculate the value of angle b
	angle_r = asin(Dotproduct(vector_P_Mi, n)/(Norm(vector_P_Mi) * Norm(n) ) );	  //calculate the value of angle r
	function_state = Solve_1_4(angle_a, angle_b, angle_r);
	if(function_state == OK)
	{
		Turn_Coordinate(refCoordinate);
	}
	else
	{
		return ERROR;
	}
    return OK;
}


void Substraction(double *a, double *b, double *diff)
{
	int i=0;
    for (i = 0; i < 3; i++) {
        diff[i] = b[i] - a[i];
    }
}                   //calculate vector coordinates

double Dotproduct(double *x, double *y)
{
    double dot = 0;
	int i=0;
    for (i = 0; i < 3; i++) {
        dot += x[i] * y[i];
    }
    return dot;
}                  //calculate dot product

double Norm(double *a)
{
    double sum = 0;
	int i = 0;
    for (i = 0; i<3; i++) {
        sum += pow(a[i], 2);
    }
    return sqrt(sum);
}                  //calculate vector Norm


int Solve_1_4(double angleA, double angleB, double angleR)
{
	double	xC = yR/tan(angleA);
	double	r = yR/sin(angleA);	//the radium of the circle
	double	xB = xC - r*cos(2*angleB-angleA);
	double	yB = -r*sin(2*angleB-angleA);

	double	B2 = xB*xB + yB*yB;		//define some parameter help to compute
	double	n = tan(angleR) * tan(angleR);
	double	v = -2*xC*xB + B2;

	double c0 = -(dMo*dMo) * B2 + n*v*v;
	double c1 = (dMo*dMo) * xB*2 + 4*n*v*xC;
	double c2 = B2 - (dMo*dMo) - 2*n*v + 4*n*xC*xC;
	double c3 = -(4*n*xC + 2*xB);
	double c4 = n+1;

	double d0, d1, d2, delta;
	double k_BM;	//define the gradient
	double x1, x2;	//store the temp result of function

	double x,xx,f,k,temp_x;
	int i=0;
	x= dMo;

	for(i=0; ; i++)
	{
		temp_x = x;	//remain unchanged
		if( temp_x < -dMo)
			break;
		f = c4*x*x*x*x + c3*x*x*x + c2*x*x + c1*x +c0;	//the value of function
		k = 4*c4*x*x*x + 3*c3*x*x + 2*c2*x + c1;		//the value of pitch
		xx = x - f/k;
		while((fabs(x-xx))>(1*10e-5) && (fabs(x) < dMo+1) && (fabs(x) > -dMo-1))
		{
			x=xx; 
			f = c4*x*x*x*x + c3*x*x*x + c2*x*x + c1*x +c0;	//the value of function
			k = 4*c4*x*x*x + 3*c3*x*x + 2*c2*x + c1;		//the value of pitch
			xx=x-f/k;
		}
		
		if((fabs(x-xx)) <= (1*10e-5))	//if jump out of the loop because of finding the right answer
		{
			sol = xx;
			break;
		}
		x = temp_x - 2.0;	//start another loop
	}


	if(sol != 0)	//compute the 1_2 answer
	{
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
		yP = k_BM * (xP-sol);
		return OK;
	}
	else
		return ERROR;
}

void Turn_Coordinate(struct reference_coordinate_s *answer)
{

	double angle;

	angle = asin(sol/dMo);
	answer->z = xP * cos(angle);
	answer->x = xP * sin(angle);
	answer->y = yP;
	answer->referenceAngle = angle;  //save the angle to struct
}


