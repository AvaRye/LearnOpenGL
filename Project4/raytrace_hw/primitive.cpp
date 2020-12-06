#include"primitive.h"
#include<sstream>
#include<cstdio>
#include<string>
#include<cmath>
#include<iostream>
#include<cstdlib>
#include <algorithm>
#define ran() ( double( rand() % 32768 ) / 32768 )

const int BEZIER_MAX_DEGREE = 5;
const int Combination[BEZIER_MAX_DEGREE + 1][BEZIER_MAX_DEGREE + 1] =
{	0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0,
	1, 2, 1, 0, 0, 0,
	1, 3, 3, 1, 0, 0,
	1, 4, 6, 4, 1, 0,
	1, 5, 10,10,5, 1
};

const int MAX_COLLIDE_TIMES = 10;
const int MAX_COLLIDE_RANDS = 10;
static void getIntersection(Vector3 ray_O, Vector3 ray_V, Vector3 N, Vector3 O, double& t, bool& front, Vector3& point);


std::pair<double, double> ExpBlur::GetXY()
{
	double x,y;
	x = ran();
	x = pow(2, x)-1;
	y = rand();
	return std::pair<double, double>(x*cos(y),x*sin(y));
}

Material::Material() {
	color = absor = Color();
	refl = refr = 0;
	diff = spec = 0;
	rindex = 0;
	drefl = 0;
	texture = NULL;
	blur = new ExpBlur();
}

void Material::Input( std::string var , std::stringstream& fin ) {
	if ( var == "color=" ) color.Input( fin );
	if ( var == "absor=" ) absor.Input( fin );
	if ( var == "refl=" ) fin >> refl;
	if ( var == "refr=" ) fin >> refr;
	if ( var == "diff=" ) fin >> diff;
	if ( var == "spec=" ) fin >> spec;
	if ( var == "drefl=" ) fin >> drefl;
	if ( var == "rindex=" ) fin >> rindex;
	if ( var == "texture=" ) {
		std::string file; fin >> file;
		texture = new Bmp;
		texture->Input( file );
	}
	if ( var == "blur=" ) {
		std::string blurname; fin >> blurname;
		if(blurname == "exp")
			blur = new ExpBlur();
	}
}

Primitive::Primitive() {
	sample = rand();
	material = new Material;
	next = NULL;
}

Primitive::Primitive( const Primitive& primitive ) {
	*this = primitive;
	material = new Material;
	*material = *primitive.material;
}

Primitive::~Primitive() {
	delete material;
}

void Primitive::Input( std::string var , std::stringstream& fin ) {
	material->Input( var , fin );
}

Sphere::Sphere() : Primitive() {
	De = Vector3( 0 , 0 , 1 );
	Dc = Vector3( 0 , 1 , 0 );
}

void Sphere::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "R=" ) fin >> R;
	if ( var == "De=" ) De.Input( fin );
	if ( var == "Dc=" ) Dc.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Sphere::Collide( Vector3 ray_O , Vector3 ray_V ) {
	ray_V = ray_V.GetUnitVector();
	Vector3 P = ray_O - O;
	double b = -P.Dot( ray_V );
	double det = b * b - P.Module2() + R * R;
	CollidePrimitive ret;

	if ( det > EPS ) {
		det = sqrt( det );
		double x1 = b - det  , x2 = b + det;

		if ( x2 < EPS ) return ret;
		if ( x1 > EPS ) {
			ret.dist = x1;
			ret.front = true;
		} else {
			ret.dist = x2;
			ret.front = false;
		} 
	} else 
		return ret;

	ret.C = ray_O + ray_V * ret.dist;
	ret.N = ( ret.C - O ).GetUnitVector();
	if ( ret.front == false ) ret.N = -ret.N;
	ret.isCollide = true;
	ret.collide_primitive = this;
	return ret;
}

Color Sphere::GetTexture(Vector3 crash_C) {
	Vector3 I = ( crash_C - O ).GetUnitVector();
	double a = acos( -I.Dot( De ) );
	double b = acos( std::min( std::max( I.Dot( Dc ) / sin( a ) , -1.0 ) , 1.0 ) );
	double u = a / PI , v = b / 2 / PI;
	if ( I.Dot( Dc * De ) < 0 ) v = 1 - v;
	return material->texture->GetSmoothColor( u , v );
}


void Plane::Input( std::string var , std::stringstream& fin ) {
	if ( var == "N=" ) N.Input( fin );
	if ( var == "R=" ) fin >> R;
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Plane::Collide( Vector3 ray_O , Vector3 ray_V ) {
	ray_V = ray_V.GetUnitVector();
	N = N.GetUnitVector();
	double d = N.Dot( ray_V );
	CollidePrimitive ret;
	if ( fabs( d ) < EPS ) return ret;
	double l = ( N * R - ray_O ).Dot( N ) / d;
	if ( l < EPS ) return ret;

	ret.dist = l;
	ret.front = ( d < 0 );
	ret.C = ray_O + ray_V * ret.dist;
	ret.N = ( ret.front ) ? N : -N;
	ret.isCollide = true;
	ret.collide_primitive = this;
	return ret;
}

Color Plane::GetTexture(Vector3 crash_C) {
	double u = crash_C.Dot( Dx ) / Dx.Module2();
	double v = crash_C.Dot( Dy ) / Dy.Module2();
	return material->texture->GetSmoothColor( u , v );
}

void Square::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Square::Collide( Vector3 ray_O , Vector3 ray_V ) {
	CollidePrimitive ret;
	Vector3 n = (Dx * Dy).GetUnitVector();
	ray_V = ray_V.GetUnitVector();
	double t;
	bool front;
	Vector3 point;
	getIntersection(ray_O, ray_V, n, O, t, front, point);
	Vector3 OP = point - O;
	if (t < EPS)
		return ret;
	if ((fabs(OP.Dot(Dx)) <= Dx.Module2()) && (fabs(OP.Dot(Dy)) <= Dy.Module2())) {
		ret.C = point;
		ret.N = n * (front ? 1 : -1);
		ret.dist = t;
		ret.collide_primitive = this;
		ret.front = front;
		ret.isCollide = true;
	}
	return ret;
}

Color Square::GetTexture(Vector3 crash_C) {
	double u = (crash_C - O).Dot( Dx ) / Dx.Module2() / 2 + 0.5;
	double v = (crash_C - O).Dot( Dy ) / Dy.Module2() / 2 + 0.5;
	return material->texture->GetSmoothColor( u , v );
}

void Cylinder::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O1=" ) O1.Input( fin );
	if ( var == "O2=" ) O2.Input( fin );
	if ( var == "R=" ) fin>>R; 
	Primitive::Input( var , fin );
}

CollidePrimitive Cylinder::Collide( Vector3 ray_O , Vector3 ray_V ) {
	CollidePrimitive ret;
	Vector3 O1O2 = (O2 - O1).GetUnitVector();
	Vector3 O1O = ray_O - O1;
	ray_V = ray_V.GetUnitVector();

	double dist1;
	double dist2;

	bool front1;
	bool front2;

	Vector3 point1;
	Vector3 point2;
	//O1
	getIntersection(ray_O, ray_V, -O1O2, O1, dist1, front1, point1);
	if ((point1 - O1).Module2() <= R * R) {
		if (!ret.isCollide || dist1 < ret.dist)
		{
			ret.dist = dist1;
			ret.C = point1;
			ret.front = front1;
			int s = front1 ? 1 : -1;
			ret.N = -s * O1O2;
			ret.isCollide = true;
			ret.collide_primitive = this;
		}
	}
	//O2
	getIntersection(ray_O, ray_V, O1O2, O2, dist2, front2, point2);
	if ((point2 - O2).Module2() <= R * R) {
		if (!ret.isCollide || dist2 < ret.dist)
		{
			ret.dist = dist2;
			ret.C = point2;
			ret.front = front2;
			int s = front2 ? 1 : -1;
			ret.N = s * O1O2;
			ret.isCollide = true;
			ret.collide_primitive = this;
		}
	}
	//cylinder
	double ax, ay, az;
	ax = (1 + O1O2.x) * (1 - O1O2.x);
	ay = (1 + O1O2.y) * (1 - O1O2.y);
	az = (1 + O1O2.z) * (1 - O1O2.z);
	double a, b, c;
	a = ax * ray_V.x * ray_V.x + ay * ray_V.y * ray_V.y + az * ray_V.z * ray_V.z;
	b = 2 * (ax * O1O.x * ray_V.x + ay * O1O.y * ray_V.y + az * O1O.z * ray_V.z);
	c = ax * O1O.x * O1O.x + ay * O1O.y * O1O.y + az * O1O.z * O1O.z - R * R;
	double delta = b * b - 4 * a * c;
	if (delta >= 0) {
		double t1 = (-b + sqrt(delta)) / (2 * a);
		double t2 = (-b - sqrt(delta)) / (2 * a);
		dist1 = BIG_DIST;
		dist2 = BIG_DIST;
		if (t1 > EPS)
		{
			dist1 = (t1 * ray_V).Module();
			point1 = ray_O + ray_V * t1;
		}
		if (t2 > EPS)
		{
			dist2 = (t2 * ray_V).Module();
			point2 = ray_O + ray_V * t2;
		}
		if (t1 > EPS && t2 > EPS) {
			front1 = t1 <= t2;
			front2 = t1 > t2;
		}
		else {
			front1 = front2 = false;
		}
		//O1P1 dot O1O2 > 0 && O2P1 dot O2O1 > 0
		if ((point1 - O1).Dot(O1O2) > 0 && (point1 - O2).Dot(-O1O2) > 0) {
			if (!ret.isCollide || dist1 < ret.dist)
			{
				ret.dist = dist1;
				ret.C = point1;
				ret.N = (O1O2 * ((point1 - O1) * O1O2)).GetUnitVector();
				ret.isCollide = true;
				ret.collide_primitive = this;
			}
		}
		if ((point2 - O1).Dot(O1O2) > 0 && (point2 - O2).Dot(-O1O2) > 0) {
			if (!ret.isCollide || dist2 < ret.dist)
			{
				ret.dist = dist2;
				ret.C = point2;
				ret.N = (O1O2 * ((point2 - O1) * O1O2)).GetUnitVector();
				ret.isCollide = true;
				ret.collide_primitive = this;
			}
		}
	}
	return ret;
}

Color Cylinder::GetTexture(Vector3 crash_C) {
	Vector3 O = (O1 + O2) / 2;
	Vector3 De = (O2 - O1).GetUnitVector();
	Vector3 Dc = (O2 - O1).GetAnVerticalVector().GetUnitVector();
	Vector3 I = (crash_C - O).GetUnitVector();
	double a = acos(-I.Dot(De));
	double b = acos(std::min(std::max(I.Dot(Dc) / sin(a), -1.0), 1.0));
	double u = a / PI, v = b / 2 / PI;
	if (I.Dot(Dc * De) < 0) v = 1 - v;
	return material->texture->GetSmoothColor(u, v);
}

void Bezier::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O1=" ) O1.Input( fin );
	if ( var == "O2=" ) O2.Input( fin );
	if ( var == "P=" ) {
		degree++;
		double newR, newZ;
		fin>>newZ>>newR;
		R.push_back(newR);
		Z.push_back(newZ);
	}
	if ( var == "Cylinder" ) {
		double maxR = 0;
		for(int i=0;i<R.size();i++)
			if(R[i] > maxR)
				maxR = R[i];
		boundingCylinder = new Cylinder(O1, O2, maxR);
		N = (O1 - O2).GetUnitVector();
		Nx = N.GetAnVerticalVector();
		Ny = N * Nx;
	}
	Primitive::Input( var , fin );
}

CollidePrimitive Bezier::Collide( Vector3 ray_O , Vector3 ray_V ) {
	CollidePrimitive ret;
	//NEED TO IMPLEMENT
	return ret;
}

Color Bezier::GetTexture(Vector3 crash_C) {
	double u = 0.5 ,v = 0.5;
	//NEED TO IMPLEMENT
	return material->texture->GetSmoothColor( u , v );
}

/**find intersection of light and plane
* ray_O origin point of light
* ray_V direction of light
* n		normal vector
* O		point
* t		out : location
* front out : if front
* point out : point location
*/
static void getIntersection(Vector3 ray_O, Vector3 ray_V, Vector3 N, Vector3 O, double& t, bool& front, Vector3& point) {
	double verticalDistance = (O - ray_O).Dot(N.GetUnitVector());
	front = verticalDistance < 0;
	t = verticalDistance / (ray_V.GetUnitVector().Dot((N).GetUnitVector()));
	point = ray_O + (ray_V.GetUnitVector() * t);
}

