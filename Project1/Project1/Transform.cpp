#include "math.h"
#include "Transform.h"

Mat3 Transform::rotate(const float degrees, const Vec3& axis)
{
    float radian = degrees * (3.1415926) / 180;
    Vec3 normalized = vecNormalize(axis);
    float x = normalized.x;
    float y = normalized.y;
    float z = normalized.z;

    Mat3 i = identityMat3();
    Mat3 a_aT = {
        {
            {x * x, x * y, x * z},
            {x * y, y * y, y * z},
            {x * z, y * z, z * z}
        }
    };
    Mat3 a_star = {
        {
            {0, -z, y},
            {z, 0, -x},
            {-y, x, 0}
        }
    };
    Mat3 component1 = scaleMatrix3(cos(radian), i);
    Mat3 component2 = scaleMatrix3((1 - cos(radian)), a_aT);
    Mat3 component3 = scaleMatrix3(sin(radian), a_star);
    Mat3 rotate = addMatrices3(component1, component2);
    rotate = addMatrices3(rotate, component3);
    return rotate;
}

void Transform::left(float degrees, Vec3& eye, Vec3& up)
{
    Mat3 r = rotate(degrees, up);

    eye = multiplyMatrixVector3(r, eye);
    up = multiplyMatrixVector3(r, up);
}

void Transform::up(float degrees, Vec3& eye, Vec3& up)
{
    Mat3 r = rotate(degrees, vecCross(eye, up));

    eye = multiplyMatrixVector3(r, eye);
    up = multiplyMatrixVector3(r, up);
}

Mat4 Transform::scale(const float& sx, const float& sy, const float& sz)
{
    Mat4 ret = {
        {
            {sx, 0, 0, 0},
            {0, sy, 0, 0},
            {0, 0, sz, 0},
            {0, 0, 0, 1}
        }
    };
    //ret = matTranspose4(ret);

    return ret;
}

Mat4 Transform::translate(const float& tx, const float& ty, const float& tz)
{
    Mat4 ret = {
        {
            {1, 0, 0, tx},
            {0, 1, 0, ty},
            {0, 0, 1, tz},
            {0, 0, 0, 1}
        }
    };
    //ret = matTranspose4(ret);

    return ret;
}

Vec3 Transform::upvector(const Vec3& up, const Vec3& zvec)
{
    Vec3 x = vecCross(up, zvec);
    Vec3 y = vecCross(zvec, x);
    Vec3 ret = vecNormalize(y);
    return ret;
}


Transform::Transform()
{

}

Transform::~Transform()
{

}
