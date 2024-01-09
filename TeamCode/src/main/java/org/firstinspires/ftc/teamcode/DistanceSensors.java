package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.CommandNotImplementedException;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@Config
public class DistanceSensors {
    public static int NUM_SENSORS = 4;
    public static double MIN_DIST_TO_DETECT = 0.02;
    private Multiplexer mux;
    private Sensor4742 us;
    private MultipleTelemetry debug;
    public DistanceSensors(HardwareMap hwMap, MultipleTelemetry debug) {
        this.debug = debug;
        mux = hwMap.get(Multiplexer.class, "mux");
        us = hwMap.get(Sensor4742.class, "us");

        // Loop over the ports activating each color sensor
        for (int i = 0; i < NUM_SENSORS; i++) {
            mux.setTarget(i);
            debug.addData("Dist-Sensor " + i, us.readDistance());
        }
    }

    // Check if there's something in front of the bot (pixel / custom)
    public boolean checkFront() {
        double avg = checkDist(0) + checkDist(1);
        return (avg / 2.0) < MIN_DIST_TO_DETECT;
    }

    // Check the distance in front of a sensor
    // Could change implementation to make it sample multiple times, wait, and get rid of outliers (depends on how good the sensors are)
    public double checkDist(int index) {
        return 0.0;
    }
}
