#pragma once
class Vector
{
public:
	float x;
	float y;
	float z;
	Vector();
	Vector(float xNew, float yNew, float zNew);
	Vector(Vector v1, Vector v2);
	Vector getNormalVector(Vector v2);
	void normalize();
	~Vector();
private:

};

