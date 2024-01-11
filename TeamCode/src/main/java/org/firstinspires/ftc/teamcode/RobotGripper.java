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
            } else if (Math.abs(delta) < SERVO_CALC_EPSILON) {
                return CRServoState.Float;
            }

            return CRServoState.Float;
        }
    }

    public static double ARM_LENGTH = 0.40; // todo: MEASURE
    public static double OFFSET_LEFT = -0.06;
    public static double OFFSET_RIGHT = -0.047;
    public static double SERVO_ROT_VAL_SPEED = 0.02;
    public static double SERVO_CALC_EPSILON = 0.02;
    private CRServo armServo, leftGripperServo, rightGripperServo;
    private double rightPower, leftPower;
    private double currentRobotFwdOffset;
    private double targetRobotFwdOffset;
    private double armAngle;
    private MultipleTelemetry debug;
    private RobotMovement movement;
    public RobotGripper(RobotMovement movement, HardwareMap hwMap, MultipleTelemetry debug) {
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

    // Set height directly. The bot will try to keep it in a straight vertical line.
    // Height: val between 0, ARM_LENGTH
    public void setArmHeight(double height) {
        double rad = Math.asin(height / ARM_LENGTH);
        armAngle = rad;
        targetRobotFwdOffset = Math.cos(rad) * ARM_LENGTH;
        movement.moveLine(targetRobotFwdOffset - currentRobotFwdOffset);
        currentRobotFwdOffset = targetRobotFwdOffset;
    }

    // Set the power position of the arm
    public void updateArmPowers(float power) {
        armServo.setPower(power);
    }

    // Open/close the claws and wait until they are completely open/closed
    // If only we didn't have to make the servos continuous. If only.
    public void setGripperTargetsWait(boolean left, boolean right) {
        //setGripperTargets(left, right);
        while (isBusy()) {
            updateServoPowers();
            Utils.sleep(10);
        }
    }

    public boolean isBusy() {
        return false;
    }

}
