#ifndef MATH_H
#define MATH_H
#include <vector>
#include <cmath>
#include <algorithm>
#include "variable.h"


float radian(float degree);
float vecDot(const Vec3& A, const Vec3& B);
float vecLength(const Vec3& A);
//return a vector with max x,y and z component from both A and B;
Vec3 vecMaxXYZ(const Vec3& A, const Vec3& B);
Vec3 vecMinXYZ(const Vec3& A, const Vec3& B);
Vec3 vecCross(const Vec3& A, const Vec3& B);
Vec3 vecNormalize(const Vec3& A);
Vec3 multiplyMatrixVector3(const Mat3& matrix, const Vec3& vec);
Vec3 multiplyMatrixVector4(const Mat4& m, const Vec3& v);
Vec3 multiplyMatrixVector4_NoDehomogenize(const Mat4& m, const Vec3& v);

Mat4 matMultiplication4(const Mat4& A, const Mat4& B);
Mat3 matMultiplication3(const Mat3& A, const Mat3& B);
Mat4 scaleMatrix4(const float scaler, const Mat4& A);
Mat3 scaleMatrix3(const float scaler, const Mat3& A);
Mat4 addMatrices4(const Mat4& A, const Mat4& B);
Mat3 addMatrices3(const Mat3& A, const Mat3& B);
Mat4 matNormalize(const Mat4& A);
Mat4 matTranspose4(const Mat4& A);
Mat3 matTranspose3(const Mat3& A);
Mat4 identityMat4();
Mat3 identityMat3();
Mat4 inverseScale(const Mat4 A);
Mat4 inverseTranslate(const Mat4 A);
Mat4 inverseTranspose(const Mat4& mat);


Vec3 transformDir(Mat4 transfMatrix, const Vec3 rayDir);
Vec3 transformPos(Mat4 transfMatrix, const Vec3 rayPos);
#endif // MATH_H

