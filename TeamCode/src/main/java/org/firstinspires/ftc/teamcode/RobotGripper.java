package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Gripper abstraction that hides the arm calculation + feedback loop and gripper buttons
@Config
public class RobotGripper {
    public enum CRServoState {
        Float,
        Forward,
        Reverse;

        public static CRServoState stateFromDelta(double delta) {
            if (delta > 0.0) {
                return CRServoState.Forward;
            } else if (delta < 0.0) {
                return CRServoState.Reverse;
            } else if (Math.abs(delta) < 0.1) {
                return CRServoState.Float;
            }

            return CRServoState.Float;
        }
    }

    public static double SERVO_ROT_VAL_SPEED = 0.02;
    private CRServo armServo, leftGripperServo, rightGripperServo;
    private double currentRight, currentLeft;
    private double targetRight, targetLeft;
    private MultipleTelemetry debug;
    public RobotGripper(HardwareMap hwMap, MultipleTelemetry debug) {
        this.debug = debug;
        leftGripperServo = hwMap.get(CRServo.class, "leftGripper");
        rightGripperServo = hwMap.get(CRServo.class, "rightGripper");
        armServo = hwMap.get(CRServo.class, "linSysServo");
    }

    public void handleGripperUpdate(Gamepad gamepad) {
        armServo.setPower(gamepad.right_stick_x);

        targetLeft = gamepad.left_bumper ? 1 : 0;
        targetRight = gamepad.right_bumper ? 1 : 0;
        double deltaLeft = currentLeft - targetLeft;
        double deltaRight = currentRight - targetRight;
        currentLeft += deltaLeft * SERVO_ROT_VAL_SPEED;
        currentRight += deltaRight * SERVO_ROT_VAL_SPEED;
        CRServoState left = CRServoState.stateFromDelta(deltaLeft);
        CRServoState right = CRServoState.stateFromDelta(deltaRight);

        // CR SERVO MODIF: power shifted by -0.06 and -0.047
        leftGripperServo.setPower(getCrPower(left, -0.06));
        rightGripperServo.setPower(getCrPower(right, -0.047));
    }

    public static double getCrPower(CRServoState state, double offset) {
        switch (state) {
            case Float:
                return offset;
            case Forward:
                return 1.0;
            case Reverse:
                return -1.0;
            default:
                return 0.0;
        }
    }
}
