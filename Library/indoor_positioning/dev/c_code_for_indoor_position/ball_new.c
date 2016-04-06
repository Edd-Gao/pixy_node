#include <stdio.h>
#include <math.h>
#define pi		3.1416		//define Pi
#define angle_a	40*pi/180	//define the three angle
#define angle_b 25*pi/180	
#define angle_r 30*pi/180
#define d_LR	20			//the distance between L and R
#define d_MO	15
#define yR		d_LR/2

double	xC = yR/tan(angle_a);
double	r = yR/sin(angle_a);	//the radium of the circle
double	xB = xC - r*cos(2*angle_b-angle_a);
double	yB = -r*sin(2*angle_b-angle_a);

double s[4]={0};	//store the result of  1_4 function
double xP[4]={0}, yP[4]={0};	//store the result of P in a certain coordinate
double xP_actual[4]={0}, yP_actual[4]={0}, zP_actual[4]={0};	//store the actual P

void solve_1_4(void)
{	
	double	B2 = xB*xB + yB*yB;		//define some parameter help to compute
	double	n = tan(angle_r) * tan(angle_r);
	double	v = -2*xC*xB + B2;

	double c0 = -(d_MO*d_MO) * B2 + n*v*v;
	double c1 = (d_MO*d_MO) * xB*2 + 4*n*v*xC;
	double c2 = B2 - (d_MO*d_MO) - 2*n*v + 4*n*xC*xC;
	double c3 = -(4*n*xC + 2*xB);
	double c4 = n+1;

	double x,xx,f,k,temp_x;
	int i=0, j=0, nn=0, flag;
	x=15.0;
//	printf("c0 = %.4f\t c1 = %.4f\t c2 = %.4f\n c3 = %.4f\t c4 = %.4f\n\n",c0,c1,c2,c3,c4);

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
			flag = 1;
			for(j=0; j<nn; j++)
			{
				if( (s[j]-xx) < (1*10e-4) )
					flag = 0;
			}
			if(flag == 1)	//judge if it is a new answer
			{
				s[nn] = xx;	//if new, store it
				nn++;	//nn is useful, it needs return
			}
		}
		x = temp_x - 3.0;	//start another loop
	}
//	return s;
 
}

void solve_1_2(void)
{
	int i;
	double k_BM[4];	//define the gradient
	double d0, d1, d2, delta;
	double x1, x2;	//store the temp result of function

	
	for(i=0; i<4; i++)
	{
		if(s[i]!=0)	//compute the 1_2 answer
		{
			printf("s = %.4f\n",s[i]);
			k_BM[i] = (-yB)/(s[i]-xB);
			d0 = xC*xC + (k_BM[i]*s[i])*(k_BM[i]*s[i]) - r*r;
			d1 = -2*(xC + k_BM[i]*k_BM[i]*s[i]);
			d2 = k_BM[i]*k_BM[i] + 1;
			delta = d1*d1 - 4*d0*d2;
			x1 = (-d1 + sqrt(delta))/(2*d2);
			x2 = (-d1 - sqrt(delta))/(2*d2);
			if((x1-xB) < 10e-5)
				xP[i] = x2;
			else
				xP[i] = x1;
			printf("xP = %.4f\n",xP[i]);
			yP[i] = k_BM[i] * (xP[i]-s[i]);
			printf("yP = %.4f\n\n",yP[i]);
		}

		
	}

}

void turn_coordinate(void)
{
	double angle_coordinate;
	int i;

	for(i=0; i<4; i++)	//turn an angle
	{
		if(s[i]!=0)
		{
			angle_coordinate = asin(s[i]/d_MO);
			xP_actual[i] = -xP[i] * cos(angle_coordinate);
			zP_actual[i] = -xP[i] * sin(angle_coordinate);
			yP_actual[i] = yP[i];		
		}
		printf("(%.4f,\t %.4f,\t %.4f)\n",xP_actual[i], yP_actual[i], zP_actual[i]);
	}
	 
}

int main()
{
	solve_1_4();
	solve_1_2();
	turn_coordinate();
}

