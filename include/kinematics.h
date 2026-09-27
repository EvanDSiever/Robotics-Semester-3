#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// MATHEMATICAL STRUCTURES: 3D VECTORS & 4x4 HOMOGENEOUS TRANSFORMATION MATRICES
// Semester 3 Robotics Project
// ============================================================================

/**
 * @brief 3D Vector in Euclidean space: v = [x, y, z]^T
 * Used to represent link translations, position vectors, and end-effector coordinates.
 */
struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    // Vector operations
    Vector3 operator+(const Vector3& o) const { return Vector3(x + o.x, y + o.y, z + o.z); }
    Vector3 operator-(const Vector3& o) const { return Vector3(x - o.x, y - o.y, z - o.z); }
    Vector3 operator*(float scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }

    // Euclidean Norm (Vector Magnitude / Length): ||v|| = sqrt(x^2 + y^2 + z^2)
    float norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    // Planar projection radius on ground horizontal XY-plane: r = sqrt(x^2 + y^2)
    float planarRadius() const {
        return sqrt(x * x + y * y);
    }

    void print(const char* label = nullptr) const {
        if (label) {
            Serial.print(label);
            Serial.print(": ");
        }
        Serial.print("[X: ");
        Serial.print(x, 2);
        Serial.print(" cm, Y: ");
        Serial.print(y, 2);
        Serial.print(" cm, Z: ");
        Serial.print(z, 2);
        Serial.println(" cm]");
    }
};

/**
 * @brief 4x4 Homogeneous Transformation Matrix
 *
 *      [ R11  R12  R13  Px ]
 *  T = [ R21  R22  R23  Py ]
 *      [ R31  R32  R33  Pz ]
 *      [  0    0    0    1 ]
 *
 * Combines 3D Rotation (upper-left 3x3) and 3D Translation (right-hand 3x1 column).
 */
struct Matrix4x4 {
    float data[4][4];

    Matrix4x4() {
        setIdentity();
    }

    void setIdentity() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                data[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // Matrix Multiplication: C = A * B
    Matrix4x4 multiply(const Matrix4x4& B) const {
        Matrix4x4 result;
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += data[r][k] * B.data[k][c];
                }
                result.data[r][c] = sum;
            }
        }
        return result;
    }

    // Extract translation position vector p = [Px, Py, Pz]^T
    Vector3 getTranslation() const {
        return Vector3(data[0][3], data[1][3], data[2][3]);
    }

    // Pretty-print the 4x4 matrix to Serial Monitor
    void print(const char* title = nullptr) const {
        if (title) {
            Serial.println(title);
        }
        for (int i = 0; i < 4; i++) {
            Serial.print("  [ ");
            for (int j = 0; j < 4; j++) {
                if (data[i][j] >= 0.0f) Serial.print(" ");
                Serial.print(data[i][j], 3);
                if (j < 3) Serial.print("  ");
            }
            Serial.println(" ]");
        }
    }
};

/**
 * @brief Represents the servo joint angles (in degrees)
 */
struct JointAngles {
    float base;
    float shoulder;
    float elbow;
    float clamp;

    JointAngles() : base(BASE_HOME_DEG), shoulder(SHOULDER_HOME_DEG), elbow(ELBOW_HOME_DEG), clamp(CLAMP_OPEN_DEG) {}
    JointAngles(float b, float s, float e, float c) : base(b), shoulder(s), elbow(e), clamp(c) {}
};

// ----------------------------------------------------------------------------
// KINEMATICS FUNCTION DECLARATIONS
// ----------------------------------------------------------------------------

/**
 * @brief Forward Kinematics (FK)
 * Calculates the end-effector 3D position vector given joint angles theta1, theta2, theta3.
 * Computes individual homogeneous transformation matrices T_01, T_12, T_23 and total T_03.
 *
 * @param baseDeg      Angle of Base servo (0° to 180°, 90° = forward)
 * @param shoulderDeg  Angle of Shoulder servo (elevation above horizontal)
 * @param elbowDeg     Angle of Elbow servo (interior angle between arm & forearm)
 * @param outTransform Optional pointer to store the resulting 4x4 transform matrix
 * @return Vector3     The End-Effector position vector [x, y, z] in centimeters
 */
Vector3 forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg, Matrix4x4* outTransform = nullptr);

/**
 * @brief Inverse Kinematics (IK)
 * Solves for joint angles (theta1, theta2, theta3) needed to reach target end-effector coordinate.
 * Uses geometric derivation (cylindrical decomposition + Law of Cosines).
 *
 * @param target     Target position vector [x, y, z] in centimeters
 * @param outAngles  Struct to store solved joint angles (in degrees)
 * @return bool      true if coordinate is reachable within workspace; false if out of bounds
 */
bool inverseKinematics(const Vector3& target, JointAngles& outAngles);

/**
 * @brief Formatted printout of Forward Kinematics results with matrices and vectors
 */
void printKinematicsReport(float baseDeg, float shoulderDeg, float elbowDeg);

#endif // KINEMATICS_H
