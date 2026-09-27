#include "kinematics.h"

#define DEG_TO_RAD_F 0.017453292519943295f
#define RAD_TO_DEG_F 57.29577951308232f

// ============================================================================
// FORWARD KINEMATICS (FK) IMPLEMENTATION
// Calculates end-effector position vector p = [x, y, z]^T and 4x4 matrix T_03
// ============================================================================
Vector3 forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg, Matrix4x4* outTransform) {
    // 1. Angle conversions to radians
    // Base servo: 90° points directly along +X axis (phi_base = 0)
    float phi_base = (baseDeg - 90.0f) * DEG_TO_RAD_F;

    // Shoulder elevation angle above horizontal
    float phi_shoulder = shoulderDeg * DEG_TO_RAD_F;

    // Forearm pitch angle: elbowDeg is the interior triangle angle between link 2 and link 3
    // When elbowDeg = 180°, forearm is completely straight with upper arm
    float phi_forearm = phi_shoulder + (elbowDeg - 180.0f) * DEG_TO_RAD_F;

    // 2. Trigonometric components in the vertical arm plane
    float cos_b = cos(phi_base);
    float sin_b = sin(phi_base);

    float cos_s = cos(phi_shoulder);
    float sin_s = sin(phi_shoulder);

    float cos_f = cos(phi_forearm);
    float sin_f = sin(phi_forearm);

    // 3. Planar link vectors (in sagittal plane)
    // Link 2 vector: L2 * [cos(phi_shoulder), sin(phi_shoulder)]
    float dr2 = L2_UPPER_ARM * cos_s;
    float dz2 = L2_UPPER_ARM * sin_s;

    // Link 3 vector: L3 * [cos(phi_forearm), sin(phi_forearm)]
    float dr3 = L3_FOREARM_CLAMP * cos_f;
    float dz3 = L3_FOREARM_CLAMP * sin_f;

    // Total planar radius from base rotation axis
    float r = dr2 + dr3;

    // 4. Cartesian position vector p = [x, y, z]^T
    float x = r * cos_b;
    float y = r * sin_b;
    float z = L1_BASE_HEIGHT + dz2 + dz3;

    Vector3 endEffectorPos(x, y, z);

    // 5. Construct Homogeneous Transformation Matrix T_03 if requested
    if (outTransform != nullptr) {
        // Pitch of the end-effector orientation: phi_forearm
        // Yaw of the end-effector orientation: phi_base
        outTransform->setIdentity();

        // Combined Rotation Matrix (R_z(phi_base) * R_y(pitch)):
        outTransform->data[0][0] = cos_b * cos_f;
        outTransform->data[0][1] = -sin_b;
        outTransform->data[0][2] = cos_b * sin_f;
        outTransform->data[0][3] = x; // Translation Px

        outTransform->data[1][0] = sin_b * cos_f;
        outTransform->data[1][1] = cos_b;
        outTransform->data[1][2] = sin_b * sin_f;
        outTransform->data[1][3] = y; // Translation Py

        outTransform->data[2][0] = -sin_f;
        outTransform->data[2][1] = 0.0f;
        outTransform->data[2][2] = cos_f;
        outTransform->data[2][3] = z; // Translation Pz

        outTransform->data[3][0] = 0.0f;
        outTransform->data[3][1] = 0.0f;
        outTransform->data[3][2] = 0.0f;
        outTransform->data[3][3] = 1.0f;
    }

    return endEffectorPos;
}

// ============================================================================
// INVERSE KINEMATICS (IK) IMPLEMENTATION
// Solves target [x, y, z] -> [theta1, theta2, theta3]
// ============================================================================
bool inverseKinematics(const Vector3& target, JointAngles& outAngles) {
    // ------------------------------------------------------------------------
    // Step 1: Base Angle (Yaw)
    // Project vector onto horizontal XY plane: phi_base = atan2(y, x)
    // ------------------------------------------------------------------------
    float phi_base = atan2(target.y, target.x);
    float baseDeg = 90.0f + (phi_base * RAD_TO_DEG_F);

    if (baseDeg < BASE_MIN_DEG || baseDeg > BASE_MAX_DEG) {
        return false; // Beyond physical base rotation limit
    }

    // ------------------------------------------------------------------------
    // Step 2: Planar Projection (Cylindrical Coordinates)
    // r = sqrt(x^2 + y^2),  z' = z - L1_BASE_HEIGHT
    // ------------------------------------------------------------------------
    float r = sqrt(target.x * target.x + target.y * target.y);
    float z_rel = target.z - L1_BASE_HEIGHT;

    // Euclidean distance D from shoulder pivot (0, L1) to target (r, z')
    float D = sqrt(r * r + z_rel * z_rel);

    // ------------------------------------------------------------------------
    // Step 3: Reachability Verification (Workspace Boundaries)
    // Triangle inequality: |L2 - L3| <= D <= (L2 + L3)
    // ------------------------------------------------------------------------
    float D_max = L2_UPPER_ARM + L3_FOREARM_CLAMP;
    float D_min = fabs(L2_UPPER_ARM - L3_FOREARM_CLAMP);

    if (D > D_max || D < D_min) {
        return false; // Target is physically unreachable
    }

    // ------------------------------------------------------------------------
    // Step 4: Elbow Angle via Law of Cosines
    // D^2 = L2^2 + L3^2 - 2 * L2 * L3 * cos(gamma)
    // cos(gamma) = (L2^2 + L3^2 - D^2) / (2 * L2 * L3)
    // ------------------------------------------------------------------------
    float cos_gamma = ((L2_UPPER_ARM * L2_UPPER_ARM) + (L3_FOREARM_CLAMP * L3_FOREARM_CLAMP) - (D * D)) /
                      (2.0f * L2_UPPER_ARM * L3_FOREARM_CLAMP);

    // Numerical clamping to avoid acos domain error with floating point drift
    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;

    float gamma = acos(cos_gamma); // Interior elbow angle in radians
    float elbowDeg = gamma * RAD_TO_DEG_F;

    // ------------------------------------------------------------------------
    // Step 5: Shoulder Angle via Law of Cosines & Geometry
    // alpha = angle of line of sight from shoulder to target: atan2(z', r)
    // beta = internal angle at shoulder: cos(beta) = (L2^2 + D^2 - L3^2) / (2 * L2 * D)
    // Shoulder elevation = alpha + beta (Elbow-Up configuration)
    // ------------------------------------------------------------------------
    float alpha = atan2(z_rel, r);

    float cos_beta = ((L2_UPPER_ARM * L2_UPPER_ARM) + (D * D) - (L3_FOREARM_CLAMP * L3_FOREARM_CLAMP)) /
                     (2.0f * L2_UPPER_ARM * D);

    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;

    float beta = acos(cos_beta);

    float phi_shoulder = alpha + beta; // Elbow-up solution
    float shoulderDeg = phi_shoulder * RAD_TO_DEG_F;

    // ------------------------------------------------------------------------
    // Step 6: Joint Limits Verification
    // ------------------------------------------------------------------------
    if (shoulderDeg < SHOULDER_MIN_DEG || shoulderDeg > SHOULDER_MAX_DEG ||
        elbowDeg < ELBOW_MIN_DEG || elbowDeg > ELBOW_MAX_DEG) {
        return false; // Solved angles exceed physical servo travel bounds
    }

    outAngles.base = baseDeg;
    outAngles.shoulder = shoulderDeg;
    outAngles.elbow = elbowDeg;

    return true;
}

// ============================================================================
// COMPREHENSIVE KINEMATICS REPORT GENERATOR
// ============================================================================
void printKinematicsReport(float baseDeg, float shoulderDeg, float elbowDeg) {
    Matrix4x4 T;
    Vector3 pos = forwardKinematics(baseDeg, shoulderDeg, elbowDeg, &T);

    Serial.println(F("\n========================================================"));
    Serial.println(F("              KINEMATICS MATHEMATICAL REPORT             "));
    Serial.println(F("========================================================"));
    
    Serial.print(F("1. JOINT ANGLES (Input):\n"));
    Serial.print(F("   - Base Theta 1     : ")); Serial.print(baseDeg, 1); Serial.println(F(" deg"));
    Serial.print(F("   - Shoulder Theta 2 : ")); Serial.print(shoulderDeg, 1); Serial.println(F(" deg"));
    Serial.print(F("   - Elbow Theta 3    : ")); Serial.print(elbowDeg, 1); Serial.println(F(" deg"));

    Serial.println(F("\n2. HOMOGENEOUS TRANSFORMATION MATRIX T (0 -> 3):"));
    Serial.println(F("     [  R11    R12    R13     Px  ]"));
    Serial.println(F("     [  R21    R22    R23     Py  ]"));
    Serial.println(F("     [  R31    R32    R33     Pz  ]"));
    Serial.println(F("     [   0      0      0       1  ]"));
    T.print();

    Serial.println(F("\n3. END-EFFECTOR POSITION VECTOR p (Output):"));
    Serial.print(F("   p = [ X: ")); Serial.print(pos.x, 2); Serial.print(F(" cm, "));
    Serial.print(F("Y: ")); Serial.print(pos.y, 2); Serial.print(F(" cm, "));
    Serial.print(F("Z: ")); Serial.print(pos.z, 2); Serial.println(F(" cm ]^T"));

    float r = sqrt(pos.x * pos.x + pos.y * pos.y);
    float dist = pos.norm();
    Serial.print(F("   - Ground Radius (r)       : ")); Serial.print(r, 2); Serial.println(F(" cm"));
    Serial.print(F("   - 3D Vector Norm (||p||)  : ")); Serial.print(dist, 2); Serial.println(F(" cm"));
    Serial.println(F("========================================================\n"));
}
