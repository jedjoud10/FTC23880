package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@I2cDeviceType
@DeviceProperties(name = "Distance Sensor 4742", xmlTag = "DistanceSensor")
public class Sensor4742 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }
    public Sensor4742(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(new I2cAddr(0x57));
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
        super.registerArmingStateCallback(false);
        this.deviceClient.setHeartbeatInterval(0);
        this.deviceClient.engage();
        this.deviceClient.write8((byte)1);
    }
    @Override
    public String getDeviceName() { return "Custom Ultrasonic Distance Sensor"; }

    public void enable() {
        deviceClient.engage();
        deviceClient.write8((byte)1);
        deviceClient.disengage();
    }

    public double readDistance(int sampleDelay, int sampleCount) {
        deviceClient.engage();

        double avg = 0.0;

        for (int i = 0; i < sampleCount; i++) {
            byte[] bytes = deviceClient.read(3);
            int micrometers = bytes[0] * 0x10000 + bytes[1] * 0x100 + bytes[2];
            double dist = DistanceUnit.METER.fromMm(micrometers / 1000.0);
            Utils.sleep(sampleDelay);
            avg += dist;
        }

        Utils.sleep(30);
        deviceClient.write8((byte)1);
        deviceClient.disengage();
        return avg / (float)sampleCount;
    }
}