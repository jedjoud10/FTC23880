package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@Config
public class DistanceChecker {
    public static double MIN_DIST_TO_DETECT_MM = 75;
    private ColorRangeSensor sensor;
    public DistanceChecker(HardwareMap hwMap) {
        sensor = hwMap.get(ColorRangeSensor.class, "color");
    }

    // Check if there's something in front of the bot (pixel / custom)
    public boolean checkFront() {
        return sensor.getDistance(DistanceUnit.MM) < MIN_DIST_TO_DETECT_MM;
    }
}
