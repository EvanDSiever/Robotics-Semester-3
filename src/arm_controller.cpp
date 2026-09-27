#include "arm_controller.h"

ArmController::ArmController() : currentAngles(BASE_HOME_DEG, SHOULDER_HOME_DEG, ELBOW_HOME_DEG, CLAMP_OPEN_DEG) {}

void ArmController::begin() {
    Serial.println(F("[HARDWARE] Attaching PWM pins to SG90 Servos..."));

    // Attach PWM Pins
    baseServo.attach(PIN_BASE_SERVO);
    shoulderServo.attach(PIN_SHOULDER_SERVO);
    elbowServo.attach(PIN_ELBOW_SERVO);
    clampServo.attach(PIN_CLAMP_SERVO);

    // Initial position write before enabling full motion
    baseServo.write(round(currentAngles.base));
    shoulderServo.write(round(currentAngles.shoulder));
    elbowServo.write(round(currentAngles.elbow));
    clampServo.write(round(currentAngles.clamp));

    delay(300);
    Serial.println(F("[HARDWARE] Servos attached and holding HOME position."));
}

void ArmController::moveToAngles(float targetBase, float targetShoulder, float targetElbow, float targetClamp, int stepDelayMs) {
    // 1. Constrain to physical servo boundaries
    targetBase = constrain(targetBase, (float)BASE_MIN_DEG, (float)BASE_MAX_DEG);
    targetShoulder = constrain(targetShoulder, (float)SHOULDER_MIN_DEG, (float)SHOULDER_MAX_DEG);
    targetElbow = constrain(targetElbow, (float)ELBOW_MIN_DEG, (float)ELBOW_MAX_DEG);

    float startClamp = currentAngles.clamp;
    float endClamp = (targetClamp >= 0.0f) ? constrain(targetClamp, (float)CLAMP_MIN_DEG, (float)CLAMP_MAX_DEG) : startClamp;

    // 2. Compute maximum angle displacement to determine number of interpolation steps
    float dBase = fabs(targetBase - currentAngles.base);
    float dShoulder = fabs(targetShoulder - currentAngles.shoulder);
    float dElbow = fabs(targetElbow - currentAngles.elbow);
    float dClamp = fabs(endClamp - startClamp);

    float maxDelta = max(dBase, max(dShoulder, max(dElbow, dClamp)));
    int steps = ceil(maxDelta);
    if (steps < 1) steps = 1;

    // 3. Smooth multi-joint linear interpolation
    JointAngles startAngles = currentAngles;

    for (int step = 1; step <= steps; step++) {
        float t = (float)step / (float)steps;

        float b = startAngles.base + t * (targetBase - startAngles.base);
        float s = startAngles.shoulder + t * (targetShoulder - startAngles.shoulder);
        float e = startAngles.elbow + t * (targetElbow - startAngles.elbow);
        float c = startClamp + t * (endClamp - startClamp);

        baseServo.write(round(b));
        shoulderServo.write(round(s));
        elbowServo.write(round(e));
        clampServo.write(round(c));

        delay(stepDelayMs);
    }

    // 4. Update internal angle record
    currentAngles.base = targetBase;
    currentAngles.shoulder = targetShoulder;
    currentAngles.elbow = targetElbow;
    currentAngles.clamp = endClamp;
}

bool ArmController::moveToPosition(const Vector3& targetPos, float targetClamp) {
    JointAngles solved;
    if (!inverseKinematics(targetPos, solved)) {
        Serial.print(F("[IK ERROR] Target coordinate "));
        targetPos.print();
        Serial.println(F("is OUT OF REACHABLE WORKSPACE or violates joint limits!"));
        return false;
    }

    Serial.print(F("[IK SOLVED] Target "));
    targetPos.print();
    Serial.print(F(" -> Angles: Base="));
    Serial.print(solved.base, 1);
    Serial.print(F("°, Shoulder="));
    Serial.print(solved.shoulder, 1);
    Serial.print(F("°, Elbow="));
    Serial.print(solved.elbow, 1);
    Serial.println(F("°"));

    moveToAngles(solved.base, solved.shoulder, solved.elbow, targetClamp);
    return true;
}

void ArmController::goHome() {
    Serial.println(F("[MOTION] Moving arm smoothly to HOME position..."));
    moveToAngles(BASE_HOME_DEG, SHOULDER_HOME_DEG, ELBOW_HOME_DEG, CLAMP_OPEN_DEG);
}

void ArmController::openClamp() {
    Serial.println(F("[CLAMP] Opening clamp..."));
    moveToAngles(currentAngles.base, currentAngles.shoulder, currentAngles.elbow, CLAMP_OPEN_DEG);
}

void ArmController::closeClamp() {
    Serial.println(F("[CLAMP] Closing clamp (gripping)..."));
    moveToAngles(currentAngles.base, currentAngles.shoulder, currentAngles.elbow, CLAMP_CLOSED_DEG);
}

void ArmController::setClampAngle(float angle) {
    moveToAngles(currentAngles.base, currentAngles.shoulder, currentAngles.elbow, angle);
}

Vector3 ArmController::getCurrentPosition() const {
    return forwardKinematics(currentAngles.base, currentAngles.shoulder, currentAngles.elbow);
}

void ArmController::runPickAndPlaceDemo() {
    Serial.println(F("\n========================================================"));
    Serial.println(F("     STARTING AUTOMATED PICK-AND-PLACE DEMONSTRATION     "));
    Serial.println(F("========================================================"));

    // Step 1: Start at Home with Clamp open
    Serial.println(F("\n[STEP 1/8] Calibrating Home Position (Clamp Open)..."));
    goHome();
    printKinematicsReport(currentAngles.base, currentAngles.shoulder, currentAngles.elbow);
    delay(1000);

    // Step 2: Move above the pickup location (pre-approach)
    Vector3 approachPickup(11.0f, 6.0f, 10.0f);
    Serial.println(F("[STEP 2/8] Moving above pickup station (Z = 10.0 cm)..."));
    moveToPosition(approachPickup);
    delay(800);

    // Step 3: Descend down to grip the target object
    Vector3 atPickup(11.0f, 6.0f, 4.5f);
    Serial.println(F("[STEP 3/8] Descending to object position (Z = 4.5 cm)..."));
    moveToPosition(atPickup);
    delay(500);

    // Step 4: Close the clamp
    Serial.println(F("[STEP 4/8] Clamping object..."));
    closeClamp();
    delay(600);

    // Step 5: Ascend with the object
    Serial.println(F("[STEP 5/8] Lifting object up (Z = 12.0 cm)..."));
    Vector3 liftPos(11.0f, 6.0f, 12.0f);
    moveToPosition(liftPos);
    delay(800);

    // Step 6: Swing base around to the drop zone
    Serial.println(F("[STEP 6/8] Rotating base vector to Drop Zone (Y = -6.0 cm)..."));
    Vector3 approachDrop(11.0f, -6.0f, 12.0f);
    moveToPosition(approachDrop);
    delay(800);

    // Step 7: Lower to place the object in the drop zone
    Vector3 atDrop(11.0f, -6.0f, 4.5f);
    Serial.println(F("[STEP 7/8] Lowering to drop location (Z = 4.5 cm)..."));
    moveToPosition(atDrop);
    delay(500);

    // Release clamp
    Serial.println(F("[STEP 8/8] Releasing object and returning to Home..."));
    openClamp();
    delay(500);

    // Ascend slightly and return Home
    moveToPosition(Vector3(11.0f, -6.0f, 10.0f));
    delay(500);
    goHome();

    Serial.println(F("\n========================================================"));
    Serial.println(F("          PICK-AND-PLACE DEMONSTRATION COMPLETE         "));
    Serial.println(F("========================================================\n"));
}
