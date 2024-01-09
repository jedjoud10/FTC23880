package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

// Wrapper around UltrasonicSensors with proper placements and utility functions
// How the sensors are placed on the robot:
// Two on the front, maybe at an angle to detect custom object more easily
@I2cDeviceType
@DeviceProperties(name = "Multiplexer", xmlTag = "MTL")
@Config
public class Multiplexer extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    public Multiplexer(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public String getDeviceName()
    {
        return "Multiplexer";
    }

    public void setTarget(int index) {
        deviceClient.engage();
        deviceClient.write8(0x0, 1 << index);
        deviceClient.disengage();
    }
}