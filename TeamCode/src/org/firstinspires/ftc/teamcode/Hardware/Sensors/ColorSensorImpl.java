package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Had to create this wrapper class because I can't simply extend every possible implementation such as BroadCom, Lynx, or AMS.
 * That's why I rewrote it to take in any possible implementation.
 * This class also provides the necessary transformations to the RGB values
 */
public class ColorSensorImpl {

    ColorSensor sensor;
    public ColorSensorImpl(ColorSensor sensor){
       this.sensor = sensor;
    }

    public double[] getRGB(){
        return new double[] {this.red(), this.green(), this.blue()};
    }

    public int red()
    {
        return sensor.red() * 256/8192;
    }
    public int green()
    {
        return sensor.green() * 256/8192;
    }
    public int blue()
    {
        return sensor.blue() * 256/8192;
    }

    public int alpha(){
        return sensor.alpha();
    }

}
