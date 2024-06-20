#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>
#include <FreeImage.h>
#include "readfile.h"
#include "RayTracer.h"
#include "variable.h"

int width;
int height;
int maxdepth = 5;
int currdepth;
string outputFile;

// Camera
Vec3 eyeinit;
Vec3 upinit;
Vec3 center;
Vec3 u;
Vec3 v;
Vec3 w;
int amountinit;
float fovy;
float fovx;
Vec3 cameraDirection;

// Transformations
vector<float> translate;
vector<float> rotate;
vector<float> scale;

float sx, sy; // the scale in x and y 
float tx, ty; // the translation in x and y
vector<Mat4> sphereInvTransf;
vector<Mat4> sphereTransf;

// Light
vector<float> ambient;
vector<float> directional;
vector<float> point;
Vec3 attenuation = Vec3(1,0,0);

// Material
vector<float> diffuse;
vector<float> specular;
vector<float> emission;
vector<float> shininess;

//Scene
Vec3*** pixel = 0;
float maxverts;
float maxvertnorms;
std::vector<Triangle> triangles;
std::vector<Vec3> trinormals;
std::vector<Sphere> spheres;
Scene scene;
std::vector<Vec3> vertices;
std::vector<pair<Vec3, Vec3>> vertexnormals;

Vec3 P;
Vec3 normal;
float t;
int curr_Tri;
int curr_Sphere;

const float infty = 1e30f;

void init_pixels() {
    pixel = new Vec3 ** [height];
    for (int i = 0; i < height; ++i) {
        pixel[i] = new Vec3*[width];
    }
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            pixel[i][j] = new Vec3(0, 0, 0);
        }
    }
    curr_Tri = -1;
    curr_Sphere = -1;

}

void delete_Pixels() {
    // Deallocate memory for the image
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; j++) {
            delete pixel[i][j];
        }
        delete[] pixel[i];
    }
    delete[] pixel;
    pixel = nullptr;
}

int main() {
    readfile("../testscenes/scene7.test");
    init_pixels();

    scene.Triangles = triangles;
    scene.Spheres = spheres;

    RayTracer* rc = new RayTracer();
    FreeImage_Initialise();
    std::vector<unsigned char> pixels(width * height * 3); 

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int idx = (i * width + j) * 3;
            pixels[idx] = pixel[height - 1- i][j]->z * 255;  // Blue
            pixels[idx + 1] = pixel[height - 1 - i][j]->y * 255;   // Green
            pixels[idx + 2] = pixel[height - 1 - i][j]->x * 255;  // Red
        }
    }

    // Convert the pixel array to a FreeImage bitmap
    FIBITMAP* img = FreeImage_ConvertFromRawBits(pixels.data(), width, height, width * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, false);

    // Save the image
    std::string fname = outputFile;
    FreeImage_Save(FIF_PNG, img, fname.c_str(), 0);

    // Cleanup
    FreeImage_Unload(img);
    FreeImage_DeInitialise();
    delete_Pixels();

    return 0;
}