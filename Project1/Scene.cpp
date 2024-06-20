#include "Scene.h"

void Scene::initScene() {
	scene = scene(triangles, spheres);
}

void Scene::deleteScene() {
    for (Triangle* triangle : scene.Triangles) {
        delete triangle; 
    }
    for (Sphere* sphere : scene.Spheres) {
        delete sphere;
    }
    scene.Triangles.clear(); // Clear the vector
    scene.Spheres.clear();
}