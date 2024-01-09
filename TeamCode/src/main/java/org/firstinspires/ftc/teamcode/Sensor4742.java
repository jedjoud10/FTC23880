package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@I2cDeviceType
@DeviceProperties(name = "Distance Sensor 4742", xmlTag = "DistanceSensor")
@Config
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
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    @Override
    public String getDeviceName() { return "Custom Ultrasonic Distance Sensor"; }

    public double readDistance() {
        deviceClient.engage();
        deviceClient.write8(0x57, 1);
        Utils.sleep(100);
        int micrometers = TypeConversion.byteArrayToInt(deviceClient.read(0x57, 3));
        double dist = DistanceUnit.METER.fromMm(micrometers / 1000.0);
        deviceClient.disengage();
        return dist;
    }
}