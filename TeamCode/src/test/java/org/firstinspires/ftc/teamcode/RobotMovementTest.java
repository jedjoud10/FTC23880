package org.firstinspires.ftc.teamcode;

import junit.framework.TestCase;

public class RobotMovementTest extends TestCase {
    public void testCalculateTravelledDistance() {
        System.out.println("Test travelled dist");
        Tuple<Double> distances = RobotMovement.calculateTravelledDistance(1.0);
        System.out.println(distances.left);
        System.out.println(distances.right);

        distances = RobotMovement.calculateTravelledDistance(-1.0);
        System.out.println(distances.left);
        System.out.println(distances.right);
    }

    public void testCalculateMotorVelocities() {
        System.out.println("Test velocities");
        Tuple<Double> velocities = RobotMovement.calculateMotorVelocities(1.0, 0.0);
        System.out.println(velocities.left);
        System.out.println(velocities.right);
        velocities = RobotMovement.calculateMotorVelocities(1.0, 1.0);
        System.out.println(velocities.left);
        System.out.println(velocities.right);
        velocities = RobotMovement.calculateMotorVelocities(0.01, 1.0);
        System.out.println(velocities.left);
        System.out.println(velocities.right);
        velocities = RobotMovement.calculateMotorVelocities(-0.01, 1.0);
        System.out.println(velocities.left);
        System.out.println(velocities.right);
    }

    public void testCalculateArcRadius() {
        System.out.println("Test arc rad");
        double radius = RobotMovement.calculateArcRadius(0.0);
        assert(Math.abs(radius) > 5000);
        radius = RobotMovement.calculateArcRadius(-1.0);
        assert(Math.abs(radius) < 0.01);
        assert(radius < 0.0);
        radius = RobotMovement.calculateArcRadius(1.0);
        assert(Math.abs(radius) < 0.01);
        assert(radius > 0.0);
    }

    public void testFetchThrottleCurve() {
        System.out.println("Test throttle curve");
        double throttle = RobotMovement.fetchThrottleCurve(0.0, 0.0, false);
        assertEquals(0.0, throttle, 0.01);

        throttle = RobotMovement.fetchThrottleCurve(0.0, 1.0, false);
        assert(throttle > 0);
        throttle = RobotMovement.fetchThrottleCurve(0.0, -1.0, false);
        assert(throttle < 0);

        throttle = RobotMovement.fetchThrottleCurve(1.0, 0.0, false);
        assert(throttle > 0);
        throttle = RobotMovement.fetchThrottleCurve(-1.0, 0.0, false);
        assert(throttle < 0);

        throttle = RobotMovement.fetchThrottleCurve(1.0, 0.0, false);
        assert(throttle > 0);
        throttle = RobotMovement.fetchThrottleCurve(1.0, 0.0, true);
        assert(throttle < 0);
    }

    public void testConvertWheelDistanceToMotorTickPos() {
        System.out.println("Test circumf to tick pos");
        int ticks = RobotMovement.convertWheelDistanceToMotorTickPos(1.0);
        System.out.format("1m = %d ticks", ticks);
        ticks = RobotMovement.convertWheelDistanceToMotorTickPos(-1.0);
        System.out.format("-1m = %d ticks", ticks);
        System.out.println();
    }
}