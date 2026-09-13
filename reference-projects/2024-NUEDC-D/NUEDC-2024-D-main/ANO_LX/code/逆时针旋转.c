#include "stdio.h"
#include "math.h"

#define PI 3.1415
#define STEP 15
#define HEIGHT 100
#define R 50
int main()
{
	float x=0,y=0,ang=0;
	while(ang<=360)
	{
		x=2*PI*ang/360;
		printf("{%4d,%4d,%4d,0},//%.0f \r\n",(int)(R*sin(2*PI*ang/360)),(int)(-1*R*cos(2*PI*ang/360)),HEIGHT,ang);
	ang+=STEP;
	}
}
