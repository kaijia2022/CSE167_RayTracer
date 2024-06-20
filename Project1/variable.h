#include <string>
#include <vector>
#include <cmath>
#define EXTERN extern 
using namespace std;

// Below are the variables used in matrix and vector operation
#ifndef VARIABLE_H
#define VARIABLE_H
struct Vec3 {
    float x, y, z;
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator*(float f) const { return Vec3(x * f, y * f, z * f); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}
};

struct Vec4 {
    float x, y, z, w;
    Vec4(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 0.0f) : x(x), y(y), z(z), w(w) {}
};

struct Mat4 {
    float elements[4][4];
};

struct Mat3 {
    float elements[3][3];
};

struct Triangle {
    Vec3 vertice1;
    Vec3 vertice2;
    Vec3 vertice3;
    Vec3 center;
    int ambientIdx = 0;
    int emissionIdx = 0;
    int diffuseIdx = 0;
    int specularIdx = 0;
    int shininessIdx = 0;
    Triangle(const Vec3 vertice1, const Vec3 vertice2, const Vec3 vertice3) : vertice1(vertice1), vertice2(vertice2), vertice3(vertice3) {}
};

struct Sphere {
    Vec3 center;
    Vec3 transformedCenter;
    Vec3 radiusScales;
    float radius;
    int ambientIdx = 0;
    int emissionIdx = 0;
    int diffuseIdx = 0;
    int specularIdx = 0;
    int shininessIdx = 0;
    int transIdx = -1;
    Sphere(const Vec3& center, const float radius) : center(center), radius(radius) {}
};

struct Scene {
    vector<Triangle> Triangles;
    vector<Sphere> Spheres;
    Scene() {}
    Scene(vector<Triangle> Triangles, vector<Sphere> Spheres) : Triangles(Triangles), Spheres(Spheres) {}
};


struct BVHNode {
    Vec3 boundMin, boundMax;
    int left, right;
    int firstTri, triCount;
    int firstSphere, sphereCount;
};

struct Color {
    float r, g, b;
    bool isAssigned;
};

// General
EXTERN int width;
EXTERN int height;
EXTERN int maxdepth;
EXTERN int currdepth;
EXTERN string outputFile;
EXTERN Vec3 P;
EXTERN float t;
EXTERN int curr_Tri;
EXTERN int curr_Sphere;
EXTERN const float infty;
EXTERN Vec3 normal;
// Camera
EXTERN Vec3 eyeinit;
EXTERN Vec3 upinit;
EXTERN Vec3 center;
EXTERN Vec3 u;
EXTERN Vec3 v;
EXTERN Vec3 w;
EXTERN int amountinit;
EXTERN float fovy;
EXTERN float fovx;
EXTERN Vec3 cameraDirection;

// Transformations
EXTERN vector<float> translate;
EXTERN vector<float> rotate;
EXTERN vector<float> scale;

EXTERN float sx, sy; // the scale in x and y 
EXTERN float tx, ty; // the translation in x and y
EXTERN vector<Mat4> sphereInvTransf;
EXTERN vector<Mat4> sphereTransf;
// Light
EXTERN vector<float> ambient;
EXTERN vector<float> directional;
EXTERN vector<float> point;
EXTERN Vec3 attenuation;

// Material
EXTERN vector<float> diffuse;
EXTERN vector<float> specular;
EXTERN vector<float> emission;
EXTERN vector<float> shininess;

//Scene
EXTERN Vec3*** pixel;
EXTERN float maxverts;
EXTERN float maxvertnorms;
EXTERN std::vector<Triangle> triangles; 
EXTERN std::vector<Vec3> trinormals;
EXTERN std::vector<Sphere> spheres;
EXTERN Scene scene;
EXTERN std::vector<Vec3> vertices;
EXTERN std::vector<pair<Vec3, Vec3>> vertexnormals;


#endif