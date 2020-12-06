#include"light.h"
#include<sstream>
#include<string>
#include<cmath>
#include<cstdlib>
#define DECOMPOSITION_DENSITY 0.3
#define ran() ( double( rand() % 32768 ) / 32768 )

Light::Light() {
	sample = rand();
	next = NULL;
	lightPrimitive = NULL;
}

void Light::Input( std::string var , std::stringstream& fin ) {
	if ( var == "color=" ) color.Input( fin );
}

void PointLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	Light::Input( var , fin );
}


double PointLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {
	Vector3 V = O - C;
	double dist = V.Module();
	for ( Primitive* now = primitive_head ; now != NULL ; now = now->GetNext() )
	{
		CollidePrimitive tmp = now->Collide(C, V);
		if ( EPS < (dist - tmp.dist) )  return 0;
	}

	return 1;
}

void SquareLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Light::Input( var , fin );
}


double SquareLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {
	int DECOMPOSITION_X = Dx.Module() / DECOMPOSITION_DENSITY;
	int DECOMPOSITION_Y = Dy.Module() / DECOMPOSITION_DENSITY;
	int totalNum = 2 * 2 * DECOMPOSITION_X * DECOMPOSITION_Y;
	int throughNum = 0;
	bool collide;
	double stepX = (double)(1.0 / DECOMPOSITION_X);
	double stepY = (double)(1.0 / DECOMPOSITION_Y);

	for (double i = -DECOMPOSITION_X; i < DECOMPOSITION_X - EPS; i++)
	{
		for (double j = -DECOMPOSITION_Y; j < DECOMPOSITION_Y - EPS; j++)
		{
			collide = false;
			Vector3 p = O + (i * stepX * Dx) + (j * stepY * Dy);
			Vector3 V = p - C;
			double dist = V.Module();
			for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
			{
				CollidePrimitive tmp = now->Collide(C, V);
				if (EPS < (dist - tmp.dist)) {
					collide = true;
					break;
				}

			}
			if (!collide)
			{
				throughNum++;
			}
		}
	}
	return 1.0 * throughNum / totalNum;
}

Primitive* SquareLight::CreateLightPrimitive()
{
	PlaneAreaLightPrimitive* res = new PlaneAreaLightPrimitive(O, Dx, Dy, color);
	lightPrimitive = res;
	return res;
}


void SphereLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "R=" ) fin>>R;
	Light::Input( var , fin );
}


double SphereLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {
	Vector3 ez = (C - O).GetUnitVector();
	Vector3 ex = ez.GetAnVerticalVector();
	Vector3 ey = ez * ex;
	int DECOMPOSITION_Z = 5; 
	int DECOMPOSITION_a = 36; 
	double stepZ = 1.0 / DECOMPOSITION_Z;
	double stepA = 1.0 / DECOMPOSITION_a;
	int totalNum = DECOMPOSITION_Z * DECOMPOSITION_a;
	int throughNum = 0;
	bool collide;
	Vector3 P;
	for (int i = 0; i < DECOMPOSITION_Z; i++)
	{
		for (int j = 0; j < DECOMPOSITION_a; j++)
		{
			collide = false;
			double z_offset = R * i * stepZ;
			double r = sqrt(R * R - z_offset * z_offset);
			double a = 2 * PI * j * stepA;
			double y_offset = r * sin(a);
			double x_offset = r * cos(a);
			P = O + ez * z_offset + ey * y_offset + ex * x_offset;
			Vector3 V = P - C;
			double dist = V.Module();
			for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
			{
				CollidePrimitive tmp = now->Collide(C, V);
				if (EPS < (dist - tmp.dist)) {
					collide = true;
					break;
				}

			}
			if (!collide)
			{
				throughNum++;
			}
		}
	}
	return 1.0 * throughNum / totalNum;
}


Primitive* SphereLight::CreateLightPrimitive()
{
	SphereLightPrimitive* res = new SphereLightPrimitive(O, R, color);
	lightPrimitive = res;
	return res;
}

