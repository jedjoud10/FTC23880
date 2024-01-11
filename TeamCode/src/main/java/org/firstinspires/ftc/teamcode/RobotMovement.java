package org.firstinspires.ftc.teamcode;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
https://www.desmos.com/calculator/c6kk0ujwvz
We actually don't need to do any of this since we normalize it at the end
¯\_(ツ)_/¯

// Convert these arc lengths to revolutions of wheel for 1 radian
double leftWheelRot = leftDist / WHEEL_CIRCUMFERENCE;
double rightWheelRot = rightDist / WHEEL_CIRCUMFERENCE;

// Convert wheel rotations to motor rotations
double leftMotorRot = leftWheelRot * MOTOR_REV_PER_WHEEL_REV;
double rightMotorRot = rightWheelRot * MOTOR_REV_PER_WHEEL_REV;
*/

// All input angles are in degrees
// All values in metric
@Config
public class RobotMovement {
    public static double LEFT_P = 10.0;
    public static double RIGHT_P = 10.0;
    public static double LEFT_I = 5.0;
    public static double RIGHT_I = 3.0;
    public static double LEFT_D = 0.0;
    public static double RIGHT_D = 0.0;
    public static double LEFT_F = 0.0;
    public static double RIGHT_F = 0.0;
    public static double LEFT_POSITION_P = 1.985;
    public static double RIGHT_POSITION_P = 2.0;
    public static double MAJOR_THROTTLE_WEIGHT = 2.0;
    public static double MINOR_THROTTLE_WEIGHT = 0.16;
    public static double POW_CURVE_EXP = 1.0;
    public static double MAIN_THROTTLE_MUL = 0.002;
    public static double DIST_MOTORS_M = 0.29;
    public static double ARC_RAD_POW = 1.5;
    public static double ARC_RAD_MAX_TURN_RADIUS = 1.5;
    public static double MIN_ARC_DEADZONE = 0.02;
    public static double WHEEL_CIRCUMFERENCE = 2 * Math.PI * 0.045;
    public static double MOTOR_REV_PER_WHEEL_REV = 3.61 * 5.23;
    public static double ENCODER_COUNTS_PER_MOTOR_REV = 28;

    // Per the REV docs, max speed of the motor is 6000 rpm
    public static double MAX_MOTOR_VELOCITY_RAD = 2.0 * Math.PI * 6000.0;

    public Tuple<DcMotorEx> motors;
    private static MultipleTelemetry debug;

    public RobotMovement(HardwareMap hwMap, MultipleTelemetry debug) {
        DcMotorEx leftMotor = hwMap.get(DcMotorEx.class, "motor3");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx rightMotor = hwMap.get(DcMotorEx.class, "motor2");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motors = new Tuple(leftMotor, rightMotor);
        this.debug = debug;
    }

    // Calculate the distance travelled by both left and right wheels for 1 radian using a specific radius
    public static Tuple<Double> calculateTravelledDistance(double radius) {
        /*
        double leftDist = radius - DIST_MOTORS_M;
        double rightDist = radius + DIST_MOTORS_M;
        return new Tuple(leftDist, rightDist);
        */
        double leftDist = Math.abs(radius) - DIST_MOTORS_M;
        double rightDist = Math.abs(radius) + DIST_MOTORS_M;

        if (radius < 0.0) {
            double temp = rightDist;
            rightDist = leftDist;
            leftDist = temp;
        }

        return new Tuple(leftDist, rightDist);
    }

    // Calculate angular velocities for the left and right motor using a specific arc radius
    // Is not really accurate when it comes to distances as everything is normalized anyways.
    // Should only be used for driving manually
    public static Tuple<Double> calculateMotorVelocities(double radius, double throttle) {
        throttle = Range.clip(throttle, -1, 1);
        Tuple<Double> distances = calculateTravelledDistance(radius);
        //debug.addData("left", distances.left);
        //debug.addData("right", distances.right);

        // Normalize speeds to make one always equal 1
        double length = Math.max(Math.abs(distances.left), Math.abs(distances.right));
        double multiplier = MAX_MOTOR_VELOCITY_RAD * throttle;
        double leftMotorVelocity = (distances.left / length) * multiplier;
        double rightMotorVelocity = (distances.right / length) * multiplier;

        // Keep track of max velocity achievable so we can map throttle from 0-1 to 0-max
        return new Tuple(leftMotorVelocity, rightMotorVelocity);
    }

    // Convert distance travelled by wheel to motor ticks
    public static int convertWheelDistanceToMotorTickPos(double distance) {
        double wheelRotations = distance / WHEEL_CIRCUMFERENCE;
        double motorRotations = wheelRotations * MOTOR_REV_PER_WHEEL_REV;
        return (int)(ENCODER_COUNTS_PER_MOTOR_REV * motorRotations);
    }

    // Arc radius calculation in meters
    // If +1m: turns robot around a 1m circle to the left
    // If -1m: turns robot around a 1m circle to the right
    // If 0: turn robot in place
    // Should approach 0 as joystick x approaches 1, -1
    // Should approach infinity as joystick x approaches 0
    public static double calculateArcRadius(double raw) {
        //float horizontalStick = horizontalStickRaw * Math.abs(horizontalStickRaw);
        //double arcRad = Math.min(ARC_RAD_G_OFFSET / Math.pow(horizontalStickRaw, 0.4), 1000) - ARC_RAD_G_OFFSET;
        //debug.addData("H.Stick Signed Squared", horizontalStick);
        //float arcRadius = -(float)(Math.min(1.0D / (horizontalStick*4+0.00001), 1000.0D));
        raw = MathUtils.clamp(-raw, -1, 1);
        double arcRad = Math.signum(raw) - 0.91 * raw;

        // allow us to use any power
        arcRad = Math.pow(Math.abs(arcRad), ARC_RAD_POW) * Math.signum(arcRad);
        arcRad *= ARC_RAD_MAX_TURN_RADIUS;

        if (Math.abs(raw) < MIN_ARC_DEADZONE) {
            arcRad = 10000;
        }

        return arcRad;
    }

    // Applies reverse modifier and minor/major controls
    // Minor: left throttle
    // Major: right throttle
    // Should only be used for driving manually
    public static double fetchThrottleCurve(double rawMajor, double rawMinor, boolean reverse) {
        double r = reverse ? -1.0 : 1.0;
        double major = Math.pow(rawMajor, POW_CURVE_EXP) * MAJOR_THROTTLE_WEIGHT;
        double minor = rawMinor * MINOR_THROTTLE_WEIGHT;
        double throttle = (major+minor)*r;
        return throttle;
    }

    // Apply velocities to the motors (velocity in rad/s)
    // Should only be used for driving manually
    public void applyVelocities(Tuple<Double> velocities) {
        motors.left.setVelocity(-velocities.left, AngleUnit.RADIANS);
        motors.right.setVelocity(-velocities.right, AngleUnit.RADIANS);
    }

    // Set the target tick position for motors and WAIT
    // Uses throttle accel to avoid wheel slipping. Could be tuned by params
    public void setTargetTickWait(Tuple<Integer> ticks) {
        motors.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.left.setTargetPosition(-ticks.left);
        motors.right.setTargetPosition(-ticks.right);
        motors.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors.left.setVelocityPIDFCoefficients(LEFT_P, LEFT_I, LEFT_D, LEFT_F);
        motors.right.setVelocityPIDFCoefficients(RIGHT_P, RIGHT_I, RIGHT_D, RIGHT_F);
        motors.left.setPositionPIDFCoefficients(LEFT_POSITION_P);
        motors.right.setPositionPIDFCoefficients(RIGHT_POSITION_P);

        while(motors.left.isBusy() && motors.right.isBusy()) {
            float percentLeft = Math.abs((float)motors.left.getCurrentPosition() / -ticks.left);
            float percentRight = Math.abs((float)motors.right.getCurrentPosition() / -ticks.right);
            debug.addData("Left: ", percentLeft);
            debug.addData("Right: ", percentRight);
            debug.update();
            Utils.sleep(10);
        }
    }

    // Move the robot using the given gamepad input (each update tick)
    public void handleMovementUpdate(Gamepad gamepad) {

        // Main robot chassis drive code
        Tuple<Double> res;
        double radius = calculateArcRadius(gamepad.left_stick_x);
        debug.addData("Arc. Rad", radius);
        double throttle = fetchThrottleCurve(gamepad.right_trigger, gamepad.left_trigger, gamepad.b);
        debug.addData("Throttle ", throttle);
        res = calculateMotorVelocities(radius, throttle);


        res.left *= MAIN_THROTTLE_MUL;
        res.right *= MAIN_THROTTLE_MUL;
        debug.addData("Angular Rate M.R (rad/s)", res.left);
        debug.addData("Angular Rate M.L (rad/s)", res.right);
        applyVelocities(res);
    }

    // Move the robot horizontally by moving back, rotating, moving forward a bit, rotating again, and moving forward
    public void moveSteppedHorizontalRaw(double firstDist, double secondDist, double thirdDist, double angle) {
        moveLine(-firstDist);
        rotateInPlace(-angle);
        moveLine(secondDist);
        rotateInPlace(angle);
        moveLine(thirdDist);
    }

    // Smarter moveSteppedHorizontalRaw that actually takes the value of y and x and angle
    public void moveHorizontal(double x, double y, double angle) {
        // Gotta use sine law for this. I FUCKING HATE SINE LAW
        // sin angle / x = sin (pi/2 - angle) / a
        // a = x sin (pi/2 - angle) / sin angle
        double rad = angle * Math.PI / 180.0;
        double a = x * Math.sin(Math.PI/2 - rad) / Math.sin(rad);
        double hyp = Math.hypot(a, x);
        moveSteppedHorizontalRaw(y, hyp, y-a, angle);
    }

    // Move the robot forward / backwards using a specific distance
    public void moveLine(double distance) {
        int targetTickLocation = convertWheelDistanceToMotorTickPos(distance);
        setTargetTickWait(new Tuple(targetTickLocation, targetTickLocation));
    }

    // Move the robot around an arbitrary arc radius and angle
    public void moveAround(double radius, double angle) {
        double rads = angle * Math.PI / 180.0;
        double leftArcLen = convertWheelDistanceToMotorTickPos(rads * (radius - DIST_MOTORS_M));
        double rightArcLen = convertWheelDistanceToMotorTickPos(rads * (radius + DIST_MOTORS_M));
        setTargetTickWait(new Tuple(leftArcLen, rightArcLen));
    }

    // Turn the robot in place with a specific angle
    public void rotateInPlace(double angle) {
        double rad = angle * Math.PI / 180.0;
        int targetTickLocation = convertWheelDistanceToMotorTickPos(DIST_MOTORS_M * rad);
        setTargetTickWait(new Tuple(-targetTickLocation / 2, targetTickLocation / 2));
    }

    // Rotate either 90 deg clockwise/counterclockwise
    public void rotate90InPlace(boolean clockwise) {
        rotateInPlace(clockwise ? 90.0 : -90.0);
    }

    // Rotate either 180 deg clockwise/counterclockwise
    public void rotate180InPlace(boolean clockwise) {
        rotateInPlace(clockwise ? 180.0 : -180.0);
    }
}
