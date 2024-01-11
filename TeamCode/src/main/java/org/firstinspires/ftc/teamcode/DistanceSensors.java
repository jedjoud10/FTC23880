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
    public static int NUM_SENSORS = 3;
    public static double MIN_DIST_TO_DETECT = 0.06;
    public static int SAMPLE_DELAY = 5;
    public static int CHECK_DIST_DELAY = 20;
    public static int SAMPLE_COUNT = 4;
    private Multiplexer mux;
    private Sensor4742 us;
    private MultipleTelemetry debug;
    public DistanceSensors(HardwareMap hwMap, MultipleTelemetry debug) {
        this.debug = debug;
        mux = hwMap.get(Multiplexer.class, "mux");
        us = hwMap.get(Sensor4742.class, "us");

        for (int i = 0; i < NUM_SENSORS; i++) {
            mux.getDeviceClient().engage();
            mux.setTargetRead(i);
            mux.getDeviceClient().disengage();

            us.getDeviceClient().engage();
            us.enable();
            us.getDeviceClient().disengage();
            mux.getDeviceClient().engage();
        }
    }

    // Check if there's something in front of the bot (pixel / custom)
    public boolean checkFront() {
        return checkDist(0) < MIN_DIST_TO_DETECT;
    }

    // Check the distance in front of a sensor
    public double checkDist(int index) {
        Utils.sleep(CHECK_DIST_DELAY);
        mux.getDeviceClient().engage();
        mux.setTargetRead(index);
        mux.getDeviceClient().disengage();

        us.getDeviceClient().engage();
        double dist = us.readDistance(SAMPLE_DELAY, SAMPLE_COUNT);
        us.getDeviceClient().disengage();
        mux.getDeviceClient().engage();
        return dist;
    }
}
