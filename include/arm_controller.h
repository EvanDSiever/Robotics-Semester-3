#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "kinematics.h"

// ============================================================================
// ROBOTIC ARM CONTROLLER CLASS
// Handles hardware PWM servo control and smooth multi-joint trajectory execution
// ============================================================================
class ArmController {
public:
    ArmController();

    /**
     * @brief Attaches PWM pins to servos and slowly moves arm to home position
     */
    void begin();

    /**
     * @brief Move arm smoothly to target joint angles with linear interpolation
     *
     * @param targetBase      Target base angle in degrees (0 - 180)
     * @param targetShoulder  Target shoulder angle in degrees (15 - 165)
     * @param targetElbow     Target elbow angle in degrees (15 - 165)
     * @param targetClamp     Target clamp angle (-1 to keep current clamp state)
     * @param stepDelayMs     Delay in milliseconds between 1-degree steps
     */
    void moveToAngles(float targetBase, float targetShoulder, float targetElbow, float targetClamp = -1.0f, int stepDelayMs = SERVO_STEP_DELAY_MS);

    /**
     * @brief Move arm smoothly to target 3D Cartesian coordinates using Inverse Kinematics
     *
     * @param targetPos    3D Vector [x, y, z] in centimeters
     * @param targetClamp  Clamp angle (-1 to keep current state)
     * @return bool        true if move succeeded; false if target was unreachable
     */
    bool moveToPosition(const Vector3& targetPos, float targetClamp = -1.0f);

    /**
     * @brief Moves arm to the calibrated HOME position
     */
    void goHome();

    /**
     * @brief Gripper controls
     */
    void openClamp();
    void closeClamp();
    void setClampAngle(float angle);

    /**
     * @brief Getter for current joint angles
     */
    JointAngles getCurrentAngles() const { return currentAngles; }

    /**
     * @brief Calculates current end-effector position vector from current servo angles
     */
    Vector3 getCurrentPosition() const;

    /**
     * @brief Automated Pick-and-Place Demonstration Routine
     */
    void runPickAndPlaceDemo();

private:
    Servo baseServo;
    Servo shoulderServo;
    Servo elbowServo;
    Servo clampServo;

    JointAngles currentAngles;
};

#endif // ARM_CONTROLLER_H
