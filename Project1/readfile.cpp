using namespace std;
#include "readfile.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <stack>
#include "variable.h"
#include "math.h"
#include "Transform.h"

void rightmultiply(const Mat4& M, stack<Mat4>& transfstack)
{
	Mat4& T = transfstack.top();
	transfstack.top() = matMultiplication4(T, M);
}

bool readvals(stringstream& s, const int numvals, vector<float>* values) {
	float temp;
	for (int i = 0; i < numvals; i++) {
		s >> temp;
		if (s.fail()) {
			cout << "Failed reading value " << i << " will skip\n";
			return false;
		}
		values->push_back(temp);
	}
	return true;
}

bool readstr(stringstream& s, const int numstr, std::vector<std::string>& strings) {
	std::string temp;
	for (int i = 0; i < numstr; i++) {
		s >> temp;
		if (s.fail()) {
			cout << "Failed reading string " << i << " will skip\n";
			return false;
		}
		strings.push_back(temp);
	}
	return true;
}

void readfile(const char* filename) {
	string str, cmd;
	ifstream in;
	in.open(filename);
	if (in.is_open()) {

		// I need to implement a matrix stack to store transforms.  
		// This is done using standard STL Templates 
		stack <Mat4> transfstack;
		stack <Mat4> rotateStack;
		stack<Mat4> translateStack;
		stack <Mat4> scaleStack;
		Mat4 id4 = { {
			{1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 1.0f}
		} };

		transfstack.push(id4);  // identity

		getline(in, str);

		while (in) {
			if ((str.find_first_not_of(" \t\r\n") != string::npos) && (str[0] != '#')) {
				stringstream s(str);
				s >> cmd;
				int i;
				vector<float> values;
				vector<string> strings;
				bool validinput;

				// General
				if (cmd == "size") {
					validinput = readvals(s, 2, &values);
					if (validinput) {
						width = static_cast<int>(values[0]);
						height = static_cast<int>(values[1]);
					}
				}
				else if (cmd == "maxdepth") {
					validinput = readvals(s, 1, &values);
					if (validinput) {
						maxdepth = (int) values[0];
					}
				}
				else if (cmd == "output") {
					validinput = readstr(s, 1, strings);
					outputFile = strings.at(0);
				}

				// Camera
				else if (cmd == "camera") {
					validinput = readvals(s, 10, &values);
					if (validinput) {
						eyeinit = Vec3(values[0], values[1], values[2]);
						center = Vec3(values[3], values[4], values[5]);
						upinit = vecNormalize(Vec3(values[6], values[7], values[8]));
						fovy = values[9];
						fovx = atan(tan(radian(fovy) / 2) * (float)width / (float)height) * 2;

						w = vecNormalize((eyeinit - center));
						u = vecNormalize(vecCross(upinit, w));
						v = vecCross(w, u);
						cameraDirection = vecCross(vecCross((center - eyeinit), upinit), (center - eyeinit));
						cameraDirection = vecNormalize(cameraDirection);
					}
				}

				// Geometry
				else if (cmd == "sphere") {
					validinput = readvals(s, 4, &values);
					Vec3 center = Vec3(values[0], values[1], values[2]);

					Vec3 transl = Vec3(transfstack.top().elements[0][3], transfstack.top().elements[1][3], transfstack.top().elements[2][3]);

					Sphere sphere = Sphere((center), values[3]); // + transl
					sphere.transformedCenter = center + transl;
					sphere.ambientIdx = ambient.size() - 3;
					sphere.emissionIdx = emission.size() - 3;
					sphere.specularIdx = specular.size() - 3;
					sphere.diffuseIdx = diffuse.size() - 3;
					sphere.shininessIdx = shininess.size() - 1;
					//sphereTransf.push_back(transfstack.top());

					Mat4 invRotation = identityMat4();
					Mat4 rotation = identityMat4();
					if (!rotateStack.empty()) {
						invRotation = matTranspose4(rotateStack.top());
						rotation = rotateStack.top();
					}

					Mat4 invsScaling = identityMat4();
					Mat4 scaling = identityMat4();
					if (!scaleStack.empty()) {
						invsScaling = inverseScale(scaleStack.top());
						scaling = scaleStack.top();
					}

					Mat4 translation = identityMat4();
					Mat4 invTranslation = identityMat4();
					if (!translateStack.empty()) {
						invTranslation = inverseTranslate(translateStack.top());
						translation = translateStack.top();
					}

					sphereInvTransf.push_back(matMultiplication4(invsScaling, matMultiplication4(invRotation, invTranslation)));
					sphereTransf.push_back(matMultiplication4(translation, matMultiplication4(rotation, scaling)));
					sphere.transIdx = sphereInvTransf.size() - 1;
					sphere.radiusScales.x = scaling.elements[0][0];
					sphere.radiusScales.y = scaling.elements[1][1];
					sphere.radiusScales.z = scaling.elements[2][2];
					spheres.push_back(sphere);
				}
				else if (cmd == "maxverts") {
					validinput = readvals(s, 1, &values);
					maxverts = values[0];
				}
				else if (cmd == "maxvernorms") {
					validinput = readvals(s, 1, &values);
					maxvertnorms = values[0];
				}
				else if (cmd == "vertex") {
					if (vertices.size() > maxverts) {
						continue;
					}
					validinput = readvals(s, 3, &values);
					Vec3 vertex = Vec3(values[0], values[1], values[2]);
					vertices.push_back(vertex);
				}
				else if (cmd == "vertexnormal") {
					if (vertexnormals.size() > maxvertnorms) {
						continue;
					}
					validinput = readvals(s, 6, &values);
					Vec3 vertex = Vec3(values[0], values[1], values[2]);
					//Vec3 normal = Vec3(values[3], values[4], values[5]);
					//vertexnormals.push_back(std::make_pair(vertex, normal));
				}
				else if (cmd == "tri") {
					validinput = readvals(s, 3, &values);
					Triangle triangle = Triangle(vertices.at(values[0]), vertices.at(values[1]), vertices.at(values[2]));
					Vec3 sum = ((vertices.at(values[0]) + vertices.at(values[1])) + vertices.at(values[2]));
					triangle.center = sum * 0.33333f;
					triangle.ambientIdx = ambient.size() - 3;
					triangle.emissionIdx = emission.size() - 3;
					triangle.specularIdx = specular.size() - 3;
					triangle.diffuseIdx = diffuse.size() - 3;
					triangle.shininessIdx = shininess.size() - 1;

					triangle.vertice1 = multiplyMatrixVector4(transfstack.top(), triangle.vertice1);
					triangle.vertice2 = multiplyMatrixVector4(transfstack.top(), triangle.vertice2);
					triangle.vertice3 = multiplyMatrixVector4(transfstack.top(), triangle.vertice3);

					triangles.push_back(triangle);
				}
				else if (cmd == "trinormal") {
					validinput = readvals(s, 3, &values);
					Vec3 trinormal = Vec3(values[0], values[1], values[2]);
					trinormals.push_back(trinormal);
				}

				// Transformations
				else if (cmd == "translate") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						Mat4 translate = Transform::translate(values[0], values[1], values[2]);
						rightmultiply(translate, transfstack);
						translateStack.push(transfstack.top());
					}
				}
				else if (cmd == "rotate") {
					validinput = readvals(s, 4, &values);
					if (validinput) {
						Vec3 axis = Vec3(values[0], values[1], values[2]);
						Mat3 rotate3 = Transform::rotate(values[3], axis);
						Mat4 rotate = {
							{
								{rotate3.elements[0][0], rotate3.elements[0][1], rotate3.elements[0][2], 0},
								{rotate3.elements[1][0], rotate3.elements[1][1], rotate3.elements[1][2], 0},
								{rotate3.elements[2][0], rotate3.elements[2][1], rotate3.elements[2][2], 0},
								{0, 0, 0, 1}
							}
						};
						rightmultiply(rotate, transfstack);
						rotateStack.push(rotate);
					}
				}
				else if (cmd == "scale") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						Mat4 scale = Transform::scale(values[0], values[1], values[2]);
						rightmultiply(scale, transfstack);
						scaleStack.push(scale);
					}
				}
				else if (cmd == "pushTransform") {
					transfstack.push(transfstack.top());
				}
				else if (cmd == "popTransform") {
					if (transfstack.size() <= 1) {
						cerr << "Stack has no elements.  Cannot Pop\n";
					}
					else {
						transfstack.pop();
						if (!rotateStack.empty()) {
							rotateStack.pop();
						}
						if (!scaleStack.empty()) {
							scaleStack.pop();
						}
					}
				}

				// Lights
				else if (cmd == "directional") {
					validinput = readvals(s, 6, &values);
					if (validinput) {
						directional.insert(directional.end(), { values[0], values[1], values[2], values[3], values[4], values[5] });
					}
				}
				else if (cmd == "point") {
					validinput = readvals(s, 6, &values);
					if (validinput) {

						point.insert(point.end(), { values[0], values[1], values[2], values[3], values[4], values[5] });
					}
				}
				else if (cmd == "ambient") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						ambient.insert(ambient.end(), { values[0], values[1], values[2] });
						//cout << "ambient[0]: " << ambient[0] << endl;
						//cout << "ambient[1]: " << ambient[1] << endl;
						//cout << "ambient[2]: " << ambient[2] << endl;
					}
				}
				else if (cmd == "attenuation") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						attenuation = Vec3(values[0], values[1], values[2]);
					}
				}

				// Materials
				else if (cmd == "diffuse") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						diffuse.insert(diffuse.end(), { values[0], values[1], values[2] });
					}
				}
				else if (cmd == "specular") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						specular.insert(specular.end(), { values[0], values[1], values[2] });
					}
				}
				else if (cmd == "emission") {
					validinput = readvals(s, 3, &values);
					if (validinput) {
						emission.insert(emission.end(), { values[0], values[1], values[2] });
					}
				}
				else if (cmd == "shininess") {
					validinput = readvals(s, 1, &values);
					shininess.push_back(values[0]);
				}
			}
			getline(in, str);
		}
	}
}