#include "math.h"
#include <iostream>

float radian(float degree) {
	return degree * (3.1415926) / 180;
}

float vecLength(const Vec3& A) {
	return sqrt(A.x * A.x + A.y * A.y + A.z * A.z);
}

Vec3 vecMaxXYZ(const Vec3& A, const Vec3& B) {
	return Vec3(
		max(A.x, B.x),
		max(A.y, B.y),
		max(A.z, B.z)
	);
}

Vec3 vecMinXYZ(const Vec3& A, const Vec3& B) {
	return Vec3(
		min(A.x,B.x),
		min(A.y, B.y),
		min(A.z, B.z)
	);
}

float vecDot(const Vec3& A, const Vec3& B) {
	return A.x * B.x + A.y * B.y + A.z * B.z;
}

Vec3 vecCross(const Vec3& A, const Vec3& B) {
	return Vec3(
		A.y * B.z - A.z * B.y,
		A.z * B.x - A.x * B.z,
		A.x * B.y - A.y * B.x
	);
}

Vec3 vecNormalize(const Vec3& A) {
	float magnitude = sqrt(A.x * A.x + A.y * A.y + A.z * A.z);
	if (magnitude == 0.0f) {
		return Vec3(0.0f, 0.0f, 0.0f);
	}
	else {
		return Vec3(A.x / magnitude, A.y / magnitude, A.z / magnitude);
	}
}

Vec3 multiplyMatrixVector3(const Mat3& matrix, const Vec3& vec) {
	Vec3 result;
	result.x = matrix.elements[0][0] * vec.x + matrix.elements[0][1] * vec.y + matrix.elements[0][2] * vec.z;
	result.y = matrix.elements[1][0] * vec.x + matrix.elements[1][1] * vec.y + matrix.elements[1][2] * vec.z;
	result.z = matrix.elements[2][0] * vec.x + matrix.elements[2][1] * vec.y + matrix.elements[2][2] * vec.z;

	return result;
}

Vec3 multiplyMatrixVector4(const Mat4& m, const Vec3& v) {
	Vec3 result;
	result.x = v.x * m.elements[0][0] + v.y * m.elements[0][1] + v.z * m.elements[0][2] + m.elements[0][3];
	result.y = v.x * m.elements[1][0] + v.y * m.elements[1][1] + v.z * m.elements[1][2] + m.elements[1][3];
	result.z = v.x * m.elements[2][0] + v.y * m.elements[2][1] + v.z * m.elements[2][2] + m.elements[2][3];
	float w = v.x * m.elements[3][0] + v.y * m.elements[3][1] + v.z * m.elements[3][2] + m.elements[3][3];

	if (w != 0.0f) {
		result.x /= w;
		result.y /= w;
		result.z /= w;
	}

	return result;
}

//only consider the 3 by  3 submatrix
Vec3 multiplyMatrixVector4_NoDehomogenize(const Mat4& m, const Vec3& v) {
	Vec3 result;
	result.x = v.x * m.elements[0][0] + v.y * m.elements[0][1] + v.z * m.elements[0][2];
	result.y = v.x * m.elements[1][0] + v.y * m.elements[1][1] + v.z * m.elements[1][2];
	result.z = v.x * m.elements[2][0] + v.y * m.elements[2][1] + v.z * m.elements[2][2];
	return result;
}

Mat4 matMultiplication4(const Mat4& A, const Mat4& B) {
	Mat4 result;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.elements[i][j] = 0.0f;
		}
	}

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			for (int k = 0; k < 4; ++k) {
				result.elements[i][j] += A.elements[i][k] * B.elements[k][j];
			}
		}
	}

	return result;
}

Mat3 matMultiplication3(const Mat3& A, const Mat3& B) {
	Mat3 result;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.elements[i][j] = 0.0f;
		}
	}

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			for (int k = 0; k < 3; ++k) {
				result.elements[i][j] += A.elements[i][k] * B.elements[k][j];
			}
		}
	}

	return result;
}

Mat4 scaleMatrix4(const float scaler, const Mat4& A) {
	Mat4 result = A;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.elements[i][j] *= scaler;
		}
	}
	return result;
}

Mat3 scaleMatrix3(const float scaler, const Mat3& A) {
	Mat3 result = A;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.elements[i][j] *= scaler;
		}
	}
	return result;
}

Mat4 addMatrices4(const Mat4& A, const Mat4& B) {
	Mat4 result;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.elements[i][j] = A.elements[i][j] + B.elements[i][j];
		}
	}

	return result;
}

Mat3 addMatrices3(const Mat3& A, const Mat3& B) {
	Mat3 result;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.elements[i][j] = A.elements[i][j] + B.elements[i][j];
		}
	}

	return result;
}

Mat4 matNormalize(const Mat4& A) {
	Mat4 result = A;

	for (int col = 0; col < 4; ++col) {
		float magnitude = 0.0f;
		for (int row = 0; row < 4; ++row) {
			magnitude += A.elements[row][col] * A.elements[row][col];
		}
		magnitude = sqrt(magnitude);

		if (magnitude != 0.0f) {
			for (int row = 0; row < 4; ++row) {
				result.elements[row][col] /= magnitude;
			}
		}
	}

	return result;
}

Mat4 matTranspose4(const Mat4& A) {
	Mat4 result;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.elements[i][j] = A.elements[j][i];
		}
	}

	return result;
}

Mat3 matTranspose3(const Mat3& A) {
	Mat3 result;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			result.elements[i][j] = A.elements[j][i];
		}
	}

	return result;
}

Mat4 identityMat4() {
	Mat4 result;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (i == j) {
				result.elements[i][j] = 1.0f;
			}
			else {
				result.elements[i][j] = 0.0f;
			}
		}
	}
	return result;
}

Mat3 identityMat3() {
	Mat3 result;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (i == j) {
				result.elements[i][j] = 1.0f;
			}
			else {
				result.elements[i][j] = 0.0f;
			}
		}
	}
	return result;
}

Mat4 inverseScale(const Mat4 A) {
	return Mat4
	{
		{
			{ 1/A.elements[0][0], 0, 0, 0 },
			{ 0, 1 / A.elements[1][1], 0, 0 },
			{ 0, 0, 1 / A.elements[2][2], 0 },
			{ 0, 0, 0, 1 }
		}
	};
}

Mat4 inverseTranslate(const Mat4 A) {
	return Mat4
	{
		{
			{ 1, 0, 0, -A.elements[0][3] },
			{ 0, 1, 0, -A.elements[1][3] },
			{ 0, 0, 1, -A.elements[2][3] },
			{ 0, 0, 0, 1 }
		}
	};
}


Vec3 transformDir(Mat4 transfMatrix, const Vec3 rayDir) {
	Vec3 result;
	result.x = rayDir.x * transfMatrix.elements[0][0] + rayDir.y * transfMatrix.elements[0][1] + rayDir.z * transfMatrix.elements[0][2];
	result.y = rayDir.x * transfMatrix.elements[1][0] + rayDir.y * transfMatrix.elements[1][1] + rayDir.z * transfMatrix.elements[1][2];
	result.z = rayDir.x * transfMatrix.elements[2][0] + rayDir.y * transfMatrix.elements[2][1] + rayDir.z * transfMatrix.elements[2][2];
	float w = rayDir.x * transfMatrix.elements[3][0] + rayDir.y * transfMatrix.elements[3][1] + rayDir.z * transfMatrix.elements[3][2];

	if (w != 0.0f) {
		result.x /= w;
		result.y /= w;
		result.z /= w;
	}

	return result;
}

Vec3 transformPos(Mat4 transfMatrix, const Vec3 rayPos) {
	Vec3 result;
	result.x = rayPos.x * transfMatrix.elements[0][0] + rayPos.y * transfMatrix.elements[0][1] + rayPos.z * transfMatrix.elements[0][2] + transfMatrix.elements[0][3];;
	result.y = rayPos.x * transfMatrix.elements[1][0] + rayPos.y * transfMatrix.elements[1][1] + rayPos.z * transfMatrix.elements[1][2] + transfMatrix.elements[1][3];;
	result.z = rayPos.x * transfMatrix.elements[2][0] + rayPos.y * transfMatrix.elements[2][1] + rayPos.z * transfMatrix.elements[2][2] + transfMatrix.elements[2][3];;
	float w = rayPos.x * transfMatrix.elements[3][0] + rayPos.y * transfMatrix.elements[3][1] + rayPos.z * transfMatrix.elements[3][2] + transfMatrix.elements[3][3];;
	if (w != 0.0f) {
		result.x /= w;
		result.y /= w;
		result.z /= w;
	}

	return result;
}

float determinant3x3(float m[3][3]) {
	return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
		m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

Mat4 inverseTranspose(const Mat4& mat) {
	Mat4 result;
	float cofactorMat[4][4];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			float minorMat[3][3];
			int mi = 0, mj = 0;
			for (int ii = 0; ii < 4; ++ii) {
				if (ii == i) continue;
				for (int jj = 0; jj < 4; ++jj) {
					if (jj == j) continue;
					minorMat[mi][mj++] = mat.elements[ii][jj];
				}
				mi++;
				mj = 0;
			}
			float minorDet = determinant3x3(minorMat);

			float sign = ((i + j) % 2 == 0) ? 1.0f : -1.0f;
			cofactorMat[i][j] = sign * minorDet;
		}
	}

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			result.elements[i][j] = cofactorMat[j][i];
		}
	}

	return result;
}