/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class MainTeleOp extends LinearOpMode {
    private RobotMovement movement;
    private Servo launchServo;
    private CRServo armServo, leftGripperServo, rightGripperServo;
    private double rightGripperPower, leftGripperPower = 0;
    private MultipleTelemetry debug = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    @Override
    public void runOpMode() {
        // Fetch main motors from hardware
        movement = new RobotMovement(hardwareMap, debug);

        // Fetch servos from hardware
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        armServo = hardwareMap.get(CRServo.class, "linSysServo");
        leftGripperServo = hardwareMap.get(CRServo.class, "leftGripper");
        rightGripperServo = hardwareMap.get(CRServo.class, "rightGripper");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armServo.setPower(gamepad2.right_stick_x);

            // CONTINUOUS SERVO MODIF: POWER SHIFTED BY -0.06
            if (gamepad2.left_bumper) {
                rightGripperPower = -1.0;
            } else if (gamepad2.left_trigger > 0.5) {
                rightGripperPower = 1.0;
            } else {
                rightGripperPower = -0.06;
            }

            if (gamepad2.right_bumper) {
                leftGripperPower = 1;
            } else if (gamepad2.right_trigger > 0.5) {
                leftGripperPower = -1;
            } else {
                leftGripperPower = -0.047;
            }

            // Left and Right grippers controls
            leftGripperServo.setPower(leftGripperPower);
            rightGripperServo.setPower(rightGripperPower);

            debug.addData("L.G Power", leftGripperPower);
            debug.addData("R.G Power", rightGripperPower);

            // Launch servo activation
            launchServo.setPosition(gamepad1.a ? 0.4 : 0.0);
            debug.addData("L.S Power", launchServo.getPosition());

            movement.handleMovementUpdate(gamepad1);
            debug.update();
        }
    }

}