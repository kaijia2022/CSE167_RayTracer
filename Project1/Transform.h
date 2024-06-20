const float pi = 3.14159265; // For portability across platforms
#include "math.h"
#include "variable.h"

class Transform
{
public:
	Transform();
	virtual ~Transform();
	float radians(const float degrees);
	static void left(float degrees, Vec3& eye, Vec3& up);
	static void up(float degrees, Vec3& eye, Vec3& up);
	static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
	static Mat4 perspective(float fovy, float aspect, float zNear, float zFar);
	static Mat3 rotate(const float degrees, const Vec3& axis);
	static Mat4 scale(const float& sx, const float& sy, const float& sz);
	static Mat4 translate(const float& tx, const float& ty, const float& tz);
	static Vec3 upvector(const Vec3& up, const Vec3& zvec);
};