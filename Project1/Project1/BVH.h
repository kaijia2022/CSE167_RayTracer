#pragma once
#ifndef BVH_H
#define BVH_H

#include "variable.h"
#include "math.h"

//acceleration structure
class BVH
{	
public:
	BVHNode* bvhNode;
	int rootIdx, nodesUsed;

	BVH();

	~BVH();

	void updateNodeBound(int nodeIdx);

	void recursiveDivide(int nodeIdx,int& axis);
};

#endif