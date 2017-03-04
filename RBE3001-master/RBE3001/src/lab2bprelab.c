#include <math.h>

float* forward_kin(float a1, float a2, float l1, float l2)
{
	a1 = a1*M_PI/180.;
	a2 = a2*M_PI/180.;
	float pos[2] = {l1*cos(a1) + l2*cos(a1+a2) ,l1*sin(a1) + l2*sin(a1+a2) };
	return pos;
}

float* inv_kin(float x, float y, float l1, float l2)
{
	float angs[2];
	float v = (x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2);
	angs[1] = atan2(sqrt(1-(2*v)),v);
	float k[2] = {l1+l2*cos(angs[1]), l2*sin(angs[1])};
	angs[0] = atan2(y,x) - atan2(k[1],k[0]);
	return angs;
}
