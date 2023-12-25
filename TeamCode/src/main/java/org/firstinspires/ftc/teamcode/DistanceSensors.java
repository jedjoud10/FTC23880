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

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@Config
public class DistanceSensors {
    public static int NUM_SENSORS = 4;
    public static double MIN_DIST_TO_DETECT = 0.02;
    static final I2cAddr MUX_ADDRESS = new I2cAddr(0x70);
    private I2cDeviceSynch muxReader;
    static final I2cAddr US_ADDRESS = new I2cAddr(0x57);
    private I2cDeviceSynch usReader;
    private MultipleTelemetry debug;
    public DistanceSensors(HardwareMap hwMap, MultipleTelemetry debug) {
        this.debug = debug;
        muxReader = hwMap.i2cDeviceSynch.get("mux");
        muxReader.engage();

        // Loop over the ports activating each color sensor
        for (int i = 0; i < NUM_SENSORS; i++) {
            // Write to given output port on the multiplexer
            muxReader.write8(0x0, 1 << i);

            usReader = hwMap.i2cDeviceSynch.get("usSensor");
            usReader.engage();

            usReader.write8(0x57, 1);
            Utils.sleep(100);
            int micrometers = TypeConversion.byteArrayToInt(usReader.read(0x57, 3));
            double dist = DistanceUnit.METER.fromMm(micrometers / 1000.0);
            debug.addData("Dist-Sensor " + i, dist);
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
        muxReader.write8(0x0, 1 << index);
        usReader.write8(0x57, 1);
        Utils.sleep(100);
        int micrometers = TypeConversion.byteArrayToInt(usReader.read(0x57, 3));
        return DistanceUnit.METER.fromMm(micrometers / 1000.0);
    }
}
