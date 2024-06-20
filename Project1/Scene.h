#pragma once
#ifndef SCENE_H
#define SCENE_H

#include "variable.h"

class Scene
{
	//initialize a new Scene
	void initScene();

	//deallocate all the dynamically allocated structs
	void deleteScene();
};
#endif
