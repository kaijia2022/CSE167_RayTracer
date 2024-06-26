#include "RayTracer.h"
#include <iostream>
RayTracer::RayTracer() {
	bvh = new BVH();
	int i, j;

	int rootIdx = bvh->bvhNode[0].firstTri;
	int leftChild = bvh->bvhNode[1].firstTri;
	int rightChild = bvh->bvhNode[2].firstTri;


	for (i = 0; i < height; i++) {
		for (j = 0; j < width; j++) {
			t = infty;
			Vec3 raydir = rayDir(i + 0.5, j + 0.5);
			//cout << "rayDirX: " << raydir.x << endl;
			curr_Tri = -1;
			curr_Sphere = -1;
			IntersectBVHNode(raydir, bvh->bvhNode, 0, eyeinit);
			int* curr_address = &curr_Tri;
			int tria = curr_Tri;
			if (curr_Tri == -1 && curr_Sphere == -1) {
				//cout << "pixel[i][j].x: " << pixel[i][j]->x << endl;
				pixel[i][j]->x = 0;
				pixel[i][j]->y = 0;
				pixel[i][j]->z = 0;
			}
			else if (curr_Sphere != -1) {
				int ambientIdx = spheres[curr_Sphere].ambientIdx;
				int diffuseIdx = spheres[curr_Sphere].diffuseIdx;
				int specularIdx = spheres[curr_Sphere].specularIdx;
				int emissionIdx = spheres[curr_Sphere].emissionIdx;

				Vec3 myAmbient = (ambient.size() > 0) ? Vec3(ambient[ambientIdx], ambient[ambientIdx + 1], ambient[ambientIdx + 2]) : Vec3(0, 0, 0);
				Vec3 mydiffuse = (diffuse.size() > 0) ? Vec3(diffuse[diffuseIdx], diffuse[diffuseIdx + 1], diffuse[diffuseIdx + 2]) : Vec3(0, 0, 0);
				Vec3 myspecular = (specular.size() > 0) ? Vec3(specular[specularIdx], specular[specularIdx + 1], specular[specularIdx + 2]) : Vec3(0, 0, 0);
				float myshininess = (shininess.size() > 0) ? shininess[spheres[curr_Sphere].shininessIdx] : 0.0f;
				Vec3 myemission = (emission.size() > 0) ? Vec3(emission[emissionIdx], emission[emissionIdx + 1], emission[emissionIdx + 2]) : Vec3(0, 0, 0);
				Vec3 color = computeColor(raydir, normal, mydiffuse, myspecular, myshininess, myAmbient, myemission);

				pixel[i][j]->x = color.x;
				pixel[i][j]->y = color.y;
				pixel[i][j]->z = color.z;
			}
			else if (curr_Tri != -1) {
				int ambientIdx = triangles[curr_Tri].ambientIdx;
				int diffuseIdx = triangles[curr_Tri].diffuseIdx;
				int specularIdx = triangles[curr_Tri].specularIdx;
				int emissionIdx = triangles[curr_Tri].emissionIdx;

				Vec3 myAmbient = (ambient.size() > 0) ? Vec3(ambient[ambientIdx], ambient[ambientIdx + 1], ambient[ambientIdx + 2]) : Vec3(0, 0, 0);
				Vec3 mydiffuse = (diffuse.size() > 0) ? Vec3(diffuse[diffuseIdx], diffuse[diffuseIdx + 1], diffuse[diffuseIdx + 2]) : Vec3(0, 0, 0);
				Vec3 myspecular = (specular.size() > 0) ? Vec3(specular[specularIdx], specular[specularIdx + 1], specular[specularIdx + 2]) : Vec3(0, 0, 0);
				float myshininess = (shininess.size() > 0) ? shininess[triangles[curr_Tri].shininessIdx] : 0.0f;
				Vec3 myemission = (emission.size() > 0) ? Vec3(emission[emissionIdx], emission[emissionIdx + 1], emission[emissionIdx + 2]) : Vec3(0, 0, 0);
				Vec3 color = computeColor(raydir, normal, mydiffuse, myspecular, myshininess, myAmbient, myemission);
				pixel[i][j]->x = color.x;
				pixel[i][j]->y = color.y;
				pixel[i][j]->z = color.z;

			}

		}
	}

}

RayTracer::~RayTracer() {
	delete bvh;
	bvh = 0;
}

bool RayTracer::rayTriangleIntersect(const Vec3& rayDir, const Triangle triangle, float& t, Vec3& P, Vec3 startPos) {

	Vec3 oldP = P;
	Vec3 norm, A, B, C;
	A = triangle.vertice1;
	B = triangle.vertice2;
	C = triangle.vertice3;
	norm = vecNormalize(vecCross((B - A), (C - A)));
	Vec3 invRayDir = rayDir * -1;
	float isinverted = vecDot(norm, invRayDir);
	if (isinverted < 0) {
		norm = norm * -1;
	}


	float curr_t = (vecDot(norm, (A - startPos)) / vecDot(norm, rayDir));
	P = startPos + rayDir * curr_t;

	Vec3 edge1 = B - A;
	Vec3 edge2 = C - A;
	Vec3 P_A = P - A;

	float edge2_2 = vecDot(edge2, edge2);
	float edge2_1 = vecDot(edge2, edge1);
	float edge2_P = vecDot(edge2, P_A);
	float edge1_1 = vecDot(edge1, edge1);
	float edge1_P = vecDot(edge1, P_A);

	float u = (edge1_1 * edge2_P - edge2_1 * edge1_P) * (1 / (edge2_2 * edge1_1 - edge2_1 * edge2_1));
	float v = (edge2_2 * edge1_P - edge2_1 * edge2_P) * (1 / (edge2_2 * edge1_1 - edge2_1 * edge2_1));

	P = oldP;
	if (u >= 0 && v >= 0 && u + v <= 1) {
		t = (curr_t < t && curr_t > 0) ? curr_t : t;
		P = (curr_t == t) ? startPos + rayDir * curr_t : P;
		if (vecDot(norm, invRayDir) < 0) {
			int zz = 1;
		}
		normal = norm;
		return 1;
	}
	else {
		return 0;
	}
}

bool RayTracer::raySphereIntersect(const Vec3& rayDir, Vec3 startPos, const int sphereIdx, float& t, Vec3& P, bool isTransformed) {
	float a, b, c, delta, t1, t2, curr_t;
	//retrieve original pos and Dir by applying the corresponding transformation mat
	Sphere sphere = spheres[sphereIdx];
	Vec3 originalPos = multiplyMatrixVector4(sphereTransf[sphere.transIdx], startPos);
	Vec3 originalDir = multiplyMatrixVector4(sphereTransf[sphere.transIdx], rayDir);

	a = vecDot(rayDir, rayDir);
	b = 2 * vecDot(rayDir, (startPos - sphere.center));
	c = vecDot((startPos - sphere.center), (startPos - sphere.center)) - pow(sphere.radius, 2);
	delta = pow(b, 2) - 4 * a * c;
	Vec3 oldP = P;
	if (delta < 0) {
		return 0;
	}
	else if (delta == 0) {
		curr_t = -b / (2 * a);
		P = startPos + rayDir * curr_t;
		normal = vecNormalize(P - sphere.center);
		if (isTransformed) {
			P = multiplyMatrixVector4(sphereTransf[sphere.transIdx], P);
			normal = multiplyMatrixVector4_NoDehomogenize(matTranspose4(sphereInvTransf[sphere.transIdx]), normal);
			float oldCurrT = curr_t;
			curr_t = vecLength(P - originalPos);
			int i = 1;
		}
		t = (curr_t < t) ? curr_t : t;
		P = (curr_t == t) ? P : oldP;
		return 1;
	}
	else {
		t1 = (-b + sqrt(delta)) / (2 * a);
		t2 = (-b - sqrt(delta)) / (2 * a);
		if (t1 > 0 && t2 < 0) {
			curr_t = t1;
			P = startPos + rayDir * curr_t;
			normal = vecNormalize(P - sphere.center);
			if (isTransformed) {
				P = multiplyMatrixVector4(sphereTransf[sphere.transIdx], P);
				normal = multiplyMatrixVector4_NoDehomogenize(matTranspose4(sphereInvTransf[sphere.transIdx]), normal);
				float oldCurrT = curr_t;
				curr_t = vecLength(P - originalPos);
				int i = 1;
			}
			t = (curr_t < t) ? curr_t : t;
			P = (curr_t == t) ? P : oldP;
		}
		else if (t1 < 0 && t2 > 0) {
			curr_t = t2;
			P = startPos + rayDir * curr_t;
			normal = vecNormalize(P - sphere.center);
			if (isTransformed) {
				P = multiplyMatrixVector4(sphereTransf[sphere.transIdx], P);
				normal = multiplyMatrixVector4_NoDehomogenize(matTranspose4(sphereInvTransf[sphere.transIdx]), normal);
				curr_t = vecLength(P - originalPos);
				int i = 1;
			}
			t = (curr_t < t) ? curr_t : t;
			P = (curr_t == t) ? P : oldP;
		}
		else if (t1 > 0 && t2 > 0) {
			curr_t = (t1 < t2) ? t1 : t2;
			P = startPos + rayDir * curr_t;
			normal = vecNormalize(P - sphere.center);
			if (vecDot(normal, rayDir * -1) > 0) {
				int yy = 1;
			}
			if (isTransformed) {
				P = multiplyMatrixVector4(sphereTransf[sphere.transIdx], P);
				normal = multiplyMatrixVector4_NoDehomogenize(matTranspose4(sphereInvTransf[sphere.transIdx]), normal);
				float oldCurrT = curr_t;
				curr_t = vecLength(P - originalPos);
				if (vecDot(normal, rayDir*-1) > 0) {
					int yy = 1;
				}
				int i = 1;
			}
			t = (curr_t < t) ? curr_t : t;
			P = (curr_t == t) ? P : oldP;
		}
		else if (t1 < 0 && t2 < 0) {
			return 0;
		}
		return 1;
	}
}

bool RayTracer::rayBoundIntersect(const Vec3& rayDir, const Vec3 boundMin, const Vec3 boundMax, Vec3 startPos) {
	float tx1 = (boundMin.x - startPos.x) / rayDir.x;
	float tx2 = (boundMax.x - startPos.x) / rayDir.x;
	//cout << "startPos.x: " << startPos.x << endl;
	//cout << "boundMin.x: " << boundMin.x << endl;
	//cout << "rayDir.x: " << rayDir.x << endl;
	float txmin = min(tx1, tx2);
	float txmax = max(tx1, tx2);
	//cout << "txmin: " << txmin << endl;
	//cout << "txmax: " << txmax << endl;
	float ty1 = (boundMin.y - startPos.y) / rayDir.y;
	float ty2 = (boundMax.y - startPos.y) / rayDir.y;
	float tymin = min(ty1, ty2);
	float tymax = max(ty1, ty2);

	float tz1 = (boundMin.z - startPos.z) / rayDir.z;
	float tz2 = (boundMax.z - startPos.z) / rayDir.z;
	float tzmin = min(tz1, tz2);
	float tzmax = max(tz1, tz2);
	//cout << "tymin: " << tymin << endl;
	//cout << "tymin: " << tymax << endl;
	Vec3 init = startPos;
	float txymin = 0.0, txymax = 0.0;
	if (txmin <= tymax && tymin <= txmax) {
		txymin = (txmin < tymin) ? txmin : tymin;
		txymax = (txmax > tymax) ? txmax : tymax;
		if (txymin <= tzmax && tzmin <= txymax) {
			return 1;
		}

	}
	int y = 1;
	return 0;
}

void RayTracer::IntersectBVHNode(const Vec3& rayDir, BVHNode* bvhNode, const int nodeIdx, Vec3 startPos) {
	BVHNode& node = bvhNode[nodeIdx];
	bool rayboundIntersects = rayBoundIntersect(rayDir, node.boundMin, node.boundMax, startPos);
	//cout << "rayboundIntersects: " << rayboundIntersects << endl;
	if (!rayboundIntersects) {
		return;
	}
	//leaf has none zero primitive count
	if ((node.triCount + node.sphereCount) != 0) {
		for (int i = 0; i < node.sphereCount; i++) {
			float original_t = t;
			Vec3 newDir = rayDir;
			Vec3 newPos = startPos;
			Vec3 oldDir = rayDir;
			Vec3 oldPos = startPos;
			Sphere sph = spheres[node.firstSphere + i];

			int invTransfIdx = spheres[node.firstSphere + i].transIdx;
			bool isTransformed = 0;
			Mat4 transmat = sphereInvTransf[invTransfIdx];
			if (invTransfIdx != -1) {
				newDir = transformDir(sphereInvTransf[invTransfIdx], rayDir);
				newPos = transformPos(sphereInvTransf[invTransfIdx], startPos);
				isTransformed = 1;
			}
			bool intersect_Sp = raySphereIntersect(newDir, newPos, node.firstSphere + i, t, P, isTransformed);



			if (intersect_Sp) {
				//curr_Tri = (t < original_t) ? -1 : curr_Tri;
				curr_Sphere = (t < original_t) ? node.firstSphere + i : curr_Sphere;
			}

		}
		for (int i = 0; i < node.triCount; i++) {
			float original_t = t;
			bool intersect_Tri = rayTriangleIntersect(rayDir, triangles[node.firstTri + i], t, P, startPos);
			//cout << "intersect_Tri: " << intersect_Tri << endl;
			if (intersect_Tri) {
				int* curr_address = &curr_Tri;
				curr_Sphere = (t < original_t) ? -1 : curr_Sphere;
				curr_Tri = (t < original_t) ? node.firstTri + i : curr_Tri;
			}

		}

	}
	else {
		IntersectBVHNode(rayDir, bvhNode, node.left, startPos);
		IntersectBVHNode(rayDir, bvhNode, node.right, startPos);
	}
}

Vec3 RayTracer::rayDir(float i, float j) {

	float alpha = tan(fovx / 2.0f) * ((j - width / 2.0f) / (width / 2.0f));
	float beta = tan(radian(fovy) / 2.0f) * ((height / 2.0f - i) / (height / 2.0f));

	Vec3 rayDir = vecNormalize(u * alpha + v * beta - w);

	return rayDir;
}

bool RayTracer::isVisible(Vec3 lightDir, Vec3 oldP) {
	t = infty;
	Vec3 dir = lightDir;
	IntersectBVHNode(dir, bvh->bvhNode, 0, oldP);
	float curr_t = t;
	if (t < infty) {
		return 0;
	}
	return 1;
}

Vec3 RayTracer::computeLight(const Vec3 lightDir, const Vec3 lightcolor, const Vec3 mynormal, const Vec3 halfVec, const Vec3 mydiffuse, const Vec3 myspecular, const float myshininess) {
	float nDotL = vecDot(mynormal, lightDir);

	Vec3 lambert = mydiffuse * lightcolor * max(nDotL, 0.0f);

	float nDotH = vecDot(mynormal, halfVec);
	Vec3 phong = myspecular * lightcolor * pow(max(nDotH, 0.0f), myshininess);

	Vec3 retval = lambert + phong;
	return retval;
}

Vec3 RayTracer::computeColor(Vec3 rayDir, Vec3 mynormal, const Vec3 mydiffuse, const Vec3 myspecular, const float myshininess, const Vec3 ambient, const Vec3 emission) {
	Vec3 color = Vec3(0, 0, 0);
	Vec3 oldP = P;
	Vec3 oldnormal = vecNormalize(mynormal);
	for (int i = 0; i < directional.size(); i += 6) {
		Vec3 lightPos = Vec3(directional[i], directional[i + 1], directional[i + 2]);
		Vec3 lightColor = Vec3(directional[i + 3], directional[i + 4], directional[i + 5]);
		Vec3 lightDir = vecNormalize(lightPos);
		Vec3 halfVec = vecNormalize(lightDir - vecNormalize(rayDir));
		bool visible = isVisible(lightDir, oldP + lightDir * EPSILON);
		if (visible) {
			color = color + computeLight(lightDir, lightColor, oldnormal, halfVec, mydiffuse, myspecular, myshininess);
		}

	}
	for (int i = 0; i < point.size(); i += 6) {
		Vec3 lightPos = Vec3(point[i], point[i + 1], point[i + 2]);
		Vec3 lightColor = Vec3(point[i + 3], point[i + 4], point[i + 5]);
		Vec3 lightDir = vecNormalize(lightPos - oldP);
		Vec3 halfVec = vecNormalize(lightDir - vecNormalize(rayDir));

		bool visible = isVisible(lightDir, oldP + lightDir * EPSILON);
		if (visible) {
			if (vecDot(oldnormal, lightDir) > 0) {
				int xx = 1;
			}
			if (vecDot(oldnormal, rayDir*-1) > 0) {
				int yy = 1;
			}
			float distance = sqrt((lightPos - oldP).x * (lightPos - oldP).x + (lightPos - oldP).y * (lightPos - oldP).y + (lightPos - oldP).z * (lightPos - oldP).z);
			float attenuation = 1.0 / (1 * distance * distance);
			color = color + computeLight(lightDir, lightColor, oldnormal, halfVec, mydiffuse, myspecular, myshininess);
		}
	}
	Vec3 col = color + ambient + emission;
	return color + ambient + emission;
}


