package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// System for pulling the robot up
@Config
public class RobotTrussPulley {
    public static double LEFT_SERVO_POWER = -1;
    public static double RIGHT_SERVO_POWER = -1;
    public static double LEFT_PULLEY_SPEED = 0.8;
    public static double RIGHT_PULLEY_SPEED = 1.0;
    private CRServo leftServo, rightServo;
    private DcMotorEx leftMotor, rightMotor;
    private MultipleTelemetry debug;
    public RobotTrussPulley(HardwareMap hwMap, MultipleTelemetry debug) {
        this.debug = debug;
        leftServo = hwMap.get(CRServo.class, "rightPulleyServo");
        rightServo = hwMap.get(CRServo.class, "leftPulleyServo");
        leftMotor = hwMap.get(DcMotorEx.class, "leftPulleyMotor");
        rightMotor = hwMap.get(DcMotorEx.class, "rightPulleyMotor");
    }

    public void handlePulleyUpdate(Gamepad gamepad) {
        double thingyLeft = (gamepad.a || gamepad.y) ? 1.0 : 0.0;
        double thingyRight = (gamepad.b || gamepad.y) ? 1.0 : 0.0;

        if (gamepad.y) {
            thingyLeft *= -1;
            thingyRight *= -1;
        }

        leftServo.setPower(LEFT_SERVO_POWER * thingyLeft);
        rightServo.setPower(RIGHT_SERVO_POWER * thingyRight);


        double thingyLeftLeft = (gamepad.dpad_down || gamepad.dpad_up) ? 1.0 : 0.0;
        double thingyRightRight = (gamepad.dpad_down || gamepad.dpad_up) ? 1.0 : 0.0;

        if (gamepad.dpad_down) {
            thingyRightRight *= -1;
            thingyLeftLeft *= -1;
        }

        if (gamepad.dpad_right) {
            thingyRightRight *= 1;
            thingyLeftLeft = 0;
        }

        if (gamepad.dpad_left) {
            thingyLeftLeft *= 1;
            thingyRightRight = 0;
        }

        rightMotor.setPower(LEFT_PULLEY_SPEED * thingyLeftLeft);
        leftMotor.setPower(RIGHT_PULLEY_SPEED * thingyRightRight);
    }
}
