package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.Utils.telemetry;


@Autonomous(name="DiagnosticAuto", group="Autonomous Linear Opmode")
//@Disabled
public class DiagnosticAuto extends LinearOpMode {

    private MecanumRobot mecanumRobot;

    private double currentPosition, currentAngle;

    public void initialize(){
        Utils.setOpMode(this);
        mecanumRobot = new MecanumRobot();
    }


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();


        telemetry.addData("started", true);
        for (int i = 1; i < 2 + 1; i++) {
            if (opModeIsActive()){
                //mecanumRobot.turn(i * 180, 1.5);
            }
        }
        telemetry.addData("started strafe", true);
        for (int i = 0; i < 8; i++) {
            if (opModeIsActive()){
                strafe(i * 45, 500, 0.0, null);
                sleep(100);
                strafe(i * 45 + 180, 500, 0.0, null);
                sleep(100);
            }
        }
    }
    /**
     * @param angle
     * @param ticks
     */
    public void strafe(double angle, int ticks, double startAngle, SyncTask task){

        System.out.println(angle + " " + ticks);


        mecanumRobot.resetMotors();                                              // Reset Motor Encoders

        double learning_rate = 0.000001;
        double radians = angle * Math.PI / 180;                     // Convert to radians
        double yFactor = Math.sin(radians);                         // Unit Circle Y
        double xFactor = Math.cos(radians);                         // Unit Circle X
        double yTicks = Math.abs(yFactor * ticks);
        double xTicks = Math.abs(xFactor * ticks);
        double distance = Math.max(yTicks, xTicks);


        // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
        double normalizeToPower = 1 / Math.max(Math.abs(xFactor), Math.abs(yFactor));
        double drive = normalizeToPower * yFactor;                 // Fill out power to a max of 1
        double strafe = normalizeToPower * xFactor;                // Fill out power to a max of 1
        double turn = 0;


        currentPosition = mecanumRobot.getPosition();
        while (mecanumRobot.getPosition() < distance && opModeIsActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            double power = Utils.powerRamp(currentPosition, distance, 0.075);

            // PID Controller
            double error = startAngle - mecanumRobot.imu.getAngle();
            turn += error * learning_rate;
            mecanumRobot.setDrivePower(drive * power, strafe * power, turn, 1);

            // Log and get new position
            currentPosition = mecanumRobot.getPosition();
            log();
        }
        mecanumRobot.setAllPower(0);
    }

    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        telemetry.addData("Error", mecanumRobot.imu.getStartAngle() - mecanumRobot.imu.getAngle());
        //telemetry.addData("touch", mecanumRobot.touchSensor.isPressed());
    }
}