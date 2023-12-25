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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MainTeleOp extends LinearOpMode {
    private static boolean BREAKING_ENABLED = false;
    private DcMotor leftMotor, rightMotor, linSysMotor;
    private Servo launchServo;
    private CRServo linSysServo, leftGripperServo, rightGripperServo;
    private float linSysServoPos = 0;
    private double rightGripperPower, leftGripperPower = 0;
    private boolean driveMode = false;
    @Override
    public void runOpMode() {
        // Fetch main motors from hardware
        leftMotor = hardwareMap.get(DcMotor.class, "motor3");
        leftMotor.setDirection(Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        linSysMotor = hardwareMap.get(DcMotor.class, "linSysMotor");

        // Test breaking
        if (BREAKING_ENABLED) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Fetch servos from hardware
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        linSysServo = hardwareMap.get(CRServo.class, "linSysServo");
        leftGripperServo = hardwareMap.get(CRServo.class, "leftGripper");
        rightGripperServo = hardwareMap.get(CRServo.class, "rightGripper");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Lin sys power", Float.toString(gamepad1.right_stick_y));
            telemetry.addData("Lin sys cur", Integer.toString(linSysMotor.getCurrentPosition()));

            // Linear system controls
            linSysMotor.setPower(gamepad2.right_stick_y);
            linSysServo.setPower(gamepad2.right_stick_x);

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

            telemetry.addData("L.G Power", Double.toString(leftGripperPower));
            telemetry.addData("R.G Power", Double.toString(rightGripperPower));

            // Launch servo activation
            launchServo.setPosition(gamepad1.a ? 0.4 : 0.0);
            telemetry.addData("L.S Power", Double.toString(launchServo.getPosition()));

            // Swap drive mode when pressing the X button
            if (gamepad1.x) {
                driveMode = !driveMode;
                sleep(1000);
            }

            // Main robot chassis drive code
            float mR, mL = 0;
            if (driveMode) {
                telemetry.addLine("NEIL CONTROL MODE");
                float reverse = gamepad1.circle ? 1.0F : -1.0F;
                telemetry.addData("Reverse", Float.toString(reverse));
                float horizontalStickRaw = -gamepad1.left_stick_x;
                float horizontalStick = horizontalStickRaw * Math.abs(horizontalStickRaw);
                telemetry.addData("H.Stick Signed Squared", Float.toString(horizontalStick));
                float arcRadius = -(float)(Math.min(1.0D / (horizontalStick*4+0.00001), 1000.0D));
                telemetry.addData("Arc. Rad", Float.toString(arcRadius));
                float throttle = arcRadius/((float)Math.pow(gamepad1.right_trigger, 3)*reverse);
                float dist = 0.5F;
                mL = (2.0F * (float)Math.PI * (arcRadius + dist)) / throttle;
                mR = (2.0F * (float)Math.PI * (arcRadius - dist)) / throttle;
            } else {
                telemetry.addLine("JED CONTROL MODE");
                mR = 0.8F * gamepad1.left_stick_x + gamepad1.left_stick_y;
                mL = 0.8F * -gamepad1.left_stick_x + gamepad1.left_stick_y;
            }

            telemetry.addData("M.L", Float.toString(mL));
            telemetry.addData("M.R", Float.toString(mR));
            leftMotor.setPower(mL);
            rightMotor.setPower(mR);

            telemetry.update();
        }
    }
}
