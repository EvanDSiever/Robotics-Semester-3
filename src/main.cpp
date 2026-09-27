#include <Arduino.h>
#include "config.h"
#include "kinematics.h"
#include "arm_controller.h"

ArmController robotArm;

// Buffer for incoming Serial commands
String inputString = "";
bool stringComplete = false;

void printBanner() {
    Serial.println(F("\n========================================================"));
    Serial.println(F("       4-DOF ROBOTIC ARM MANIPULATOR CONTROLLER         "));
    Serial.println(F("           Semester 3 Robotics & Kinematics             "));
    Serial.println(F("========================================================"));
    Serial.println(F("PWM Pin Mapping:"));
    Serial.println(F("  - Base Joint (Yaw)     : Digital Pin 9"));
    Serial.println(F("  - Shoulder Joint       : Digital Pin 3"));
    Serial.println(F("  - Elbow Joint          : Digital Pin 11"));
    Serial.println(F("  - Clamp (End-Effector) : Digital Pin 5"));
    Serial.println(F("Physical Link Lengths:"));
    Serial.print(F("  - L1 (Base Height)     : ")); Serial.print(L1_BASE_HEIGHT); Serial.println(F(" cm"));
    Serial.print(F("  - L2 (Upper Arm)       : ")); Serial.print(L2_UPPER_ARM); Serial.println(F(" cm"));
    Serial.print(F("  - L3 (Forearm + Clamp) : ")); Serial.print(L3_FOREARM_CLAMP); Serial.println(F(" cm"));
    Serial.println(F("Type 'HELP' for available serial commands."));
    Serial.println(F("========================================================\n"));
}

void printHelp() {
    Serial.println(F("\n--- SERIAL MONITOR INTERACTIVE COMMANDS ---"));
    Serial.println(F("1. FK <base> <shoulder> <elbow>"));
    Serial.println(F("   Move arm to joint angles (in degrees) and display 4x4 Transformation Matrix & Vector."));
    Serial.println(F("   Example: FK 90 60 120"));
    Serial.println(F("2. IK <x> <y> <z>"));
    Serial.println(F("   Solve Inverse Kinematics for target coordinate in cm and move arm."));
    Serial.println(F("   Example: IK 12.0 0.0 8.0"));
    Serial.println(F("3. HOME"));
    Serial.println(F("   Move arm back to calibrated HOME position (90°, 90°, 90°)."));
    Serial.println(F("4. CLAMP <OPEN|CLOSE> (or GRIP <OPEN|CLOSE>)"));
    Serial.println(F("   Control the wrist gripper end-effector."));
    Serial.println(F("   Example: CLAMP CLOSE"));
    Serial.println(F("5. STATUS"));
    Serial.println(F("   Print current joint angles and end-effector 3D position vector."));
    Serial.println(F("6. DEMO"));
    Serial.println(F("   Run automated Pick-and-Place sequence demonstrating FK and IK in real-time."));
    Serial.println(F("7. THEORY"));
    Serial.println(F("   Print the robotics theory summary (Vectors, Matrices, End-Effector)."));
    Serial.println(F("-------------------------------------------\n"));
}

void printTheory() {
    Serial.println(F("\n========================================================"));
    Serial.println(F("         ROBOTICS THEORY & MATHEMATICAL MODEL           "));
    Serial.println(F("========================================================"));
    Serial.println(F("1. VECTORS (Euclidean 3D Position Vectors):"));
    Serial.println(F("   Each joint link is modeled as a 3D vector."));
    Serial.println(F("   The End-Effector position vector p is the vector sum:"));
    Serial.println(F("   p = r_base + r_shoulder + r_elbow"));
    Serial.println(F("   ||p|| = sqrt(x^2 + y^2 + z^2) gives the direct distance from base."));
    Serial.println(F(""));
    Serial.println(F("2. HOMOGENEOUS TRANSFORMATION MATRICES (4x4):"));
    Serial.println(F("   T = [ R(3x3)   d(3x1) ]"));
    Serial.println(F("       [ 0(1x3)     1    ]"));
    Serial.println(F("   - R(3x3) is the rotation matrix expressing joint orientation."));
    Serial.println(F("   - d(3x1) is the translation vector [x, y, z]^T to the next joint."));
    Serial.println(F("   Total chain transformation: T_total = T_01 * T_12 * T_23."));
    Serial.println(F(""));
    Serial.println(F("3. FORWARD KINEMATICS (FK):"));
    Serial.println(F("   Given joint angles (theta1, theta2, theta3):"));
    Serial.println(F("   - Cylindrical projection radius r = L2*cos(phi2) + L3*cos(phi3)"));
    Serial.println(F("   - Height z = L1 + L2*sin(phi2) + L3*sin(phi3)"));
    Serial.println(F("   - Cartesian coordinates: x = r*cos(phi1), y = r*sin(phi1)"));
    Serial.println(F(""));
    Serial.println(F("4. INVERSE KINEMATICS (IK):"));
    Serial.println(F("   Given desired target coordinates (x, y, z):"));
    Serial.println(F("   - Base yaw angle: theta1 = atan2(y, x) + 90°"));
    Serial.println(F("   - Planar distance D = sqrt(r^2 + (z - L1)^2)"));
    Serial.println(F("   - Elbow angle via Law of Cosines:"));
    Serial.println(F("     cos(gamma) = (L2^2 + L3^2 - D^2) / (2 * L2 * L3)"));
    Serial.println(F("   - Shoulder elevation via Law of Cosines:"));
    Serial.println(F("     theta2 = atan2(z - L1, r) + acos((L2^2 + D^2 - L3^2) / (2 * L2 * D))"));
    Serial.println(F("========================================================\n"));
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // Convert leading command word to uppercase
    int spaceIdx = cmd.indexOf(' ');
    String action = (spaceIdx == -1) ? cmd : cmd.substring(0, spaceIdx);
    action.toUpperCase();

    if (action == "HELP") {
        printHelp();
    } else if (action == "HOME") {
        robotArm.goHome();
        robotArm.getCurrentPosition().print("Current End-Effector Vector");
    } else if (action == "STATUS") {
        JointAngles a = robotArm.getCurrentAngles();
        Serial.print(F("Current Angles: Base=")); Serial.print(a.base, 1);
        Serial.print(F("°, Shoulder=")); Serial.print(a.shoulder, 1);
        Serial.print(F("°, Elbow=")); Serial.print(a.elbow, 1);
        Serial.print(F("°, Clamp=")); Serial.print(a.clamp, 1);
        Serial.println(F("°"));
        printKinematicsReport(a.base, a.shoulder, a.elbow);
    } else if (action == "THEORY") {
        printTheory();
    } else if (action == "DEMO") {
        robotArm.runPickAndPlaceDemo();
    } else if (action == "CLAMP" || action == "GRIP") {
        if (spaceIdx != -1) {
            String sub = cmd.substring(spaceIdx + 1);
            sub.trim();
            sub.toUpperCase();
            if (sub == "OPEN") {
                robotArm.openClamp();
            } else if (sub == "CLOSE") {
                robotArm.closeClamp();
            } else {
                float deg = sub.toFloat();
                robotArm.setClampAngle(deg);
            }
        } else {
            Serial.println(F("Usage: CLAMP OPEN or CLAMP CLOSE"));
        }
    } else if (action == "FK") {
        // Syntax: FK <base> <shoulder> <elbow>
        if (spaceIdx == -1) {
            Serial.println(F("Usage: FK <base_deg> <shoulder_deg> <elbow_deg>"));
            return;
        }
        String rest = cmd.substring(spaceIdx + 1);
        rest.trim();

        int s1 = rest.indexOf(' ');
        if (s1 == -1) {
            Serial.println(F("Usage: FK <base_deg> <shoulder_deg> <elbow_deg>"));
            return;
        }
        float b = rest.substring(0, s1).toFloat();
        String rest2 = rest.substring(s1 + 1);
        rest2.trim();

        int s2 = rest2.indexOf(' ');
        if (s2 == -1) {
            Serial.println(F("Usage: FK <base_deg> <shoulder_deg> <elbow_deg>"));
            return;
        }
        float s = rest2.substring(0, s2).toFloat();
        float e = rest2.substring(s2 + 1).toFloat();

        Serial.print(F("[COMMAND FK] Moving to angles: Base=")); Serial.print(b);
        Serial.print(F("°, Shoulder=")); Serial.print(s);
        Serial.print(F("°, Elbow=")); Serial.print(e);
        Serial.println(F("°..."));

        robotArm.moveToAngles(b, s, e);
        printKinematicsReport(b, s, e);
    } else if (action == "IK") {
        // Syntax: IK <x> <y> <z>
        if (spaceIdx == -1) {
            Serial.println(F("Usage: IK <x_cm> <y_cm> <z_cm>"));
            return;
        }
        String rest = cmd.substring(spaceIdx + 1);
        rest.trim();

        int s1 = rest.indexOf(' ');
        if (s1 == -1) {
            Serial.println(F("Usage: IK <x_cm> <y_cm> <z_cm>"));
            return;
        }
        float x = rest.substring(0, s1).toFloat();
        String rest2 = rest.substring(s1 + 1);
        rest2.trim();

        int s2 = rest2.indexOf(' ');
        if (s2 == -1) {
            Serial.println(F("Usage: IK <x_cm> <y_cm> <z_cm>"));
            return;
        }
        float y = rest2.substring(0, s2).toFloat();
        float z = rest2.substring(s2 + 1).toFloat();

        Vector3 target(x, y, z);
        Serial.print(F("[COMMAND IK] Solving target: "));
        target.print();

        if (robotArm.moveToPosition(target)) {
            JointAngles a = robotArm.getCurrentAngles();
            printKinematicsReport(a.base, a.shoulder, a.elbow);
        }
    } else {
        Serial.print(F("Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'. Type 'HELP' for a list of valid commands."));
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        ; // Wait for serial port to connect (needed for native USB)
    }

    delay(500);
    printBanner();
    robotArm.begin();
    printHelp();
}

void loop() {
    // Read serial characters
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                stringComplete = true;
            }
        } else {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        processCommand(inputString);
        inputString = "";
        stringComplete = false;
    }
}
