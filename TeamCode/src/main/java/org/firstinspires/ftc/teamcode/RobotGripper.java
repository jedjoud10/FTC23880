package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
            } else if (Math.abs(delta) < SERVO_CALC_EPSILON) {
                return CRServoState.Float;
            }

            return CRServoState.Float;
        }
    }

    public static double OFFSET_LEFT = -0.06;
    public static double OFFSET_RIGHT = -0.047;
    public static double SERVO_CALC_EPSILON = 0.02;
    public CRServo armServo, leftGripperServo, rightGripperServo;
    public double rightPower, leftPower;
    private double currentRobotFwdOffset;
    private double targetRobotFwdOffset;
    private Telemetry debug;
    private RobotMovement movement;
    public RobotGripper(RobotMovement movement, HardwareMap hwMap, Telemetry debug) {
        this.debug = debug;
        this.movement = movement;
        leftGripperServo = hwMap.get(CRServo.class, "leftGripper");
        rightGripperServo = hwMap.get(CRServo.class, "rightGripper");
        armServo = hwMap.get(CRServo.class, "armServo");
    }

    public void handleGripperUpdate(Gamepad gamepad) {
        //setArmHeight(gamepad.right_stick_y);
        leftPower = gamepad.left_bumper ? 1.0 : 0.0 - gamepad.left_trigger;
        rightPower = gamepad.right_bumper ? 1.0 : 0.0 - gamepad.right_trigger;
        updateServoPowers();
        updateArmPowers(gamepad.right_stick_y);
    }

    public void updateServoPowers() {
        leftGripperServo.setPower(leftPower + OFFSET_LEFT);
        rightGripperServo.setPower(rightPower + OFFSET_RIGHT);
    }

    public void updateArmPowers(float power) {
        armServo.setPower(power);
    }
}
