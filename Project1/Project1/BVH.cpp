#include "BVH.h"
#include <iostream>

BVH::BVH() {
	bvhNode = new BVHNode[(spheres.size() + triangles.size()) * 2 - 1];
	rootIdx = 0;
	nodesUsed = 1;
	BVHNode& rootNode = bvhNode[rootIdx];
	rootNode.left = 0;
	rootNode.right = 0;
	rootNode.firstTri = 0;
	rootNode.triCount = triangles.size();
	rootNode.firstSphere = 0;
	rootNode.sphereCount = spheres.size();
	updateNodeBound(rootIdx);
	int axis = 0;
	recursiveDivide(rootIdx, axis);
}



BVH::~BVH() {
	delete[] bvhNode;
}

void BVH::updateNodeBound(int nodeIdx) { //ok
	BVHNode& node = bvhNode[nodeIdx];
	node.boundMin = Vec3(infty, infty, infty);
	node.boundMax = Vec3(-infty, -infty, -infty);
	int firstTri = node.firstTri;
	for (int i = 0; i < node.triCount; i++) {
		Triangle& currentTri = triangles.at(firstTri + i);
		node.boundMin = vecMinXYZ(node.boundMin, currentTri.vertice1);
		node.boundMin = vecMinXYZ(node.boundMin, currentTri.vertice2);
		node.boundMin = vecMinXYZ(node.boundMin, currentTri.vertice3);

		node.boundMax = vecMaxXYZ(node.boundMax, currentTri.vertice1);
		node.boundMax = vecMaxXYZ(node.boundMax, currentTri.vertice2);
		node.boundMax = vecMaxXYZ(node.boundMax, currentTri.vertice3);
	}
	int firstSphere = node.firstSphere;
	for (int i = 0; i < node.sphereCount; i++) {
		Sphere& currentSphere = spheres.at(firstSphere + i);
		//find the longest scaled radius
		float maxScale = max(currentSphere.radius * currentSphere.radiusScales.x, 
			currentSphere.radius * currentSphere.radiusScales.y);
		maxScale = max(maxScale, currentSphere.radius * currentSphere.radiusScales.z);
		Vec3 maxScaledRadius = Vec3(maxScale, maxScale, maxScale);

		node.boundMin = vecMinXYZ(node.boundMin, (currentSphere.transformedCenter - maxScaledRadius));
		node.boundMax = vecMaxXYZ(node.boundMax, (currentSphere.transformedCenter + maxScaledRadius));
	}

}
void swapTri(int i, int j) {

	swap(triangles[i], triangles[j]);

}

void swapSphere(int i, int j) {
	swap(spheres[i], spheres[j]);
}

void BVH::recursiveDivide(int nodeIdx,int& axis) {
	BVHNode& node = bvhNode[nodeIdx];
	if ((node.triCount + node.sphereCount) < 2) return;
	Vec3 range = (node.boundMax - node.boundMin);

	float posToSplit;
	switch (axis)
	{
	case 0:
		posToSplit = node.boundMin.x + 0.5 * range.x;
		axis = 1;
		break;
	case 1:
		posToSplit = node.boundMin.y + 0.5 * range.y;
		axis = 2;
		break;
	case 2:
		posToSplit = node.boundMin.z + 0.5 * range.z;
		axis = 0;
		break;
	}

	vector<Triangle> tr = triangles;
	int tri_i = node.firstTri;
	int tri_j = tri_i + node.triCount - 1;
	while (tri_i <= tri_j) {
		float coord;
		if (axis == 0) {
			coord = triangles[tri_i].center.x;
		}
		else if (axis == 1) {
			coord = triangles[tri_i].center.y;
		}
		else if (axis == 2) {
			coord = triangles[tri_i].center.z;
		}
		
		if (coord < posToSplit) {
			tri_i++;
		}
		else {
			swapTri(tri_i, tri_j);
			tri_j--;
		}
	}
	int sp_i = node.firstSphere;
	int sp_j = sp_i + node.sphereCount - 1;
	while (sp_i <= sp_j) {
		float coord;
		if (axis == 0) {
			coord = spheres[sp_i].transformedCenter.x;
		}
		else if (axis == 1) {
			coord = spheres[sp_i].transformedCenter.y;
		}
		else if (axis == 2) {
			coord = spheres[sp_i].transformedCenter.z;
		}

		if (coord < posToSplit) {
			sp_i++;
		}
		else {
			swapSphere(sp_i, sp_j);
			sp_j--;
		}
	}

	int leftCount_tri = tri_i - node.firstTri;
	int leftCount_sp = sp_i - node.firstSphere;

	if (node.triCount != 0 || node.sphereCount != 0) {
		if (leftCount_tri + leftCount_sp == 0 || leftCount_tri+ leftCount_sp == node.triCount + node.sphereCount) return;
	}

	int leftIdx = nodesUsed;
	nodesUsed++;
	int rightIdx = nodesUsed;
	nodesUsed++;

	bvhNode[leftIdx].firstTri = node.firstTri;
	bvhNode[leftIdx].firstSphere = node.firstSphere;
	bvhNode[leftIdx].triCount = leftCount_tri;
	bvhNode[leftIdx].sphereCount = leftCount_sp;

	bvhNode[rightIdx].firstTri = tri_i;
	bvhNode[rightIdx].firstSphere = sp_i;
	bvhNode[rightIdx].triCount = node.triCount-leftCount_tri;
	bvhNode[rightIdx].sphereCount = node.sphereCount - leftCount_sp;

	node.left = leftIdx;
	node.right = rightIdx;
	node.triCount = 0;
	node.sphereCount = 0;

	updateNodeBound(leftIdx);
	updateNodeBound(rightIdx);

	recursiveDivide(leftIdx,axis);
	recursiveDivide(rightIdx,axis);

}

