#include <vector>
#include "Vector.h"

class Plain{

public:
	Plain(Vector vec1, Vector vec2);
	~Plain();
	Vector Plain::projectToPlain(Vector vecToProj);
	bool Plain::isOnFront(Vector vec);
private:

	Vector vector1;
	Vector vector2;
	Vector normal;
};