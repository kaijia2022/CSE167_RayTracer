#pragma once
#include "variable.h"
#include "math.h"
#include <cmath>
#include "BVH.h"

using namespace std;

class RayTracer
{
private:
	BVH* bvh;
	const float EPSILON = 1e-5;
public:
	RayTracer();

	~RayTracer();

	bool rayTriangleIntersect(const Vec3 &rayDir, const Triangle triangle, float &t, Vec3& P, Vec3 startPos);

	bool raySphereIntersect(const Vec3& rayDir, Vec3 startPos, const int sphereIdx, float& t, Vec3& P, bool isTransformed);

	void IntersectBVHNode(const Vec3& rayDir, BVHNode* bvhNode, const int nodeIdx, Vec3 startPos);

	bool rayBoundIntersect(const Vec3& ray, const Vec3 boundMin, const Vec3 boundMax, Vec3 startPos);

	bool isVisible(Vec3 lightDir, Vec3 oldP, float timeTolightSource);

	bool intersectsShape(Vec3 rayDir, Vec3 oldP);

	Vec3 computeLight(const Vec3 lightDir, const Vec3 lightcolor, const Vec3 normal, const Vec3 halfVec, const Vec3 mydiffuse, const Vec3 myspecular, const float myshininess);

	Vec3 computeColor(Vec3 rayDir, Vec3 mynormal, const Vec3 mydiffuse, const Vec3 myspecular, const float myshininess, const Vec3 ambient, const Vec3 emission);

	//return ray
	Vec3 rayDir(float i, float j);
};

