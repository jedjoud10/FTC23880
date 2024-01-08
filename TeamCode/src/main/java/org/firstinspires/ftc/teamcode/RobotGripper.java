package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    public static double SERVO_ROT_VAL_SPEED = 0.02;
    public static double SERVO_CALC_EPSILON = 0.02;
    private CRServo armServo, leftGripperServo, rightGripperServo;
    private double currentRight, currentLeft;
    private double targetRight, targetLeft;
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
        setArmHeight(gamepad.right_stick_y);
        setGripperTargets(gamepad.left_bumper, gamepad.right_bumper);
        updateServoPowers();
        updateArmPowersPID();
    }

    public void setGripperTargets(boolean left, boolean right) {
        targetLeft = left ? 1.0 : 0.0;
        targetRight = right ? 1.0 : 0.0;
    }

    public void updateServoPowers() {
        double deltaLeft = currentLeft - targetLeft;
        double deltaRight = currentRight - targetRight;
        currentLeft += deltaLeft * SERVO_ROT_VAL_SPEED;
        currentRight += deltaRight * SERVO_ROT_VAL_SPEED;

        // CR SERVO MODIF: power shifted by -0.06 and -0.047
        double leftPower = getCrPower(CRServoState.stateFromDelta(deltaLeft), -0.06);
        double rightPower = getCrPower(CRServoState.stateFromDelta(deltaRight), -0.047);
        leftGripperServo.setPower(leftPower);
        rightGripperServo.setPower(rightPower);
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

    // Custom PID control for the big arm servo as it uses a gear
    // for reduction and we can't use the internal angle stuff
    public void updateArmPowersPID() {
    }

    // Open/close the claws and wait until they are completely open/closed
    // If only we didn't have to make the servos continuous. If only.
    public void setGripperTargetsWait(boolean left, boolean right) {
        setGripperTargets(left, right);
        while (isBusy()) {
            updateServoPowers();
            Utils.sleep(10);
        }
    }

    public boolean isBusy() {
        double deltaLeft = currentLeft - targetLeft;
        double deltaRight = currentRight - targetRight;
        return Math.abs(deltaLeft) > SERVO_CALC_EPSILON || Math.abs(deltaRight) > SERVO_CALC_EPSILON;
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
