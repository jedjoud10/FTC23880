package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@Config
public class DistanceChecker {
    public static double MIN_DIST_TO_DETECT_MM = 75;
    public static int SAMPLE_COUNT = 5;
    public static int SAMPLE_DELAY_MS = 50;
    private ColorRangeSensor sensor;
    private Telemetry debug;
    public DistanceChecker(HardwareMap hwMap, Telemetry telemetry) {
        sensor = hwMap.get(ColorRangeSensor.class, "color");
        this.debug = telemetry;
    }

    // Check if there's something in front of the bot (pixel / custom)
    // Do multiple checks to be absolutely SURE of our measurements
    public boolean checkFront() {
        double avg = 0.0;

        debug.addLine("Check front");
        debug.update();
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            Utils.sleep(SAMPLE_DELAY_MS);
            avg += sensor.getDistance(DistanceUnit.MM);
        }

        boolean res = (avg / SAMPLE_COUNT) < MIN_DIST_TO_DETECT_MM;

        debug.addData("Result: ", res);
        debug.update();

        return res;
    }
}
