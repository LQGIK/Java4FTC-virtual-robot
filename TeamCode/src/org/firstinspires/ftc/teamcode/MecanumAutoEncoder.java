package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import net.waring.java4ftc.ops.SyncTask;
import net.waring.java4ftc.utilities.Utils;
import org.firstinspires.ftc.teamcode.Utilities.IMU;


@Autonomous(name="Mecanum Encoder", group="Autonomous Linear Opmode")
//@Disabled
public class MecanumAutoEncoder extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private IMU imu;
    private ColorSensor colorSensor;


    public void initialize() {
        // Motors
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);


        // IMU (Inertial Measurement Unit)
        Utils.setHardwareMap(hardwareMap);
        imu = new IMU("imu");

        // Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    }


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        for (int i = 1; i < 2 + 1; i++) {
            if (opModeIsActive()) {
                turn(i * 180, 0.5);
            }
        }

        for (int i = 0; i < 8; i++) {
            if (opModeIsActive()) {
                strafe(i * 45, 5000, 0.0, null);
                strafe(i * 45 + 180, 5000, 0.0, null);
            }
        }
    }

    /**
     * @param angle
     * @param ticks
     */
    public void strafe(double angle, int ticks, double startAngle, SyncTask task) {

        System.out.println(angle + " " + ticks);


        resetMotors();                                              // Reset Motor Encoders


        double learning_rate = 0.000000001;
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


        double position = getPosition();
        while (position < distance && opModeIsActive()) {

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            double power = powerRamp(position, distance, 0.3);


            // PID Controller
            double error = startAngle - imu.getAngle();
            turn += error * learning_rate;
            setDrivePower(drive * power, strafe * power, 0.0);


            // Retrieve new position
            position = getPosition();


            // Telemetry
            telemetry.addData("IMU", imu.getAngle());
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Power", power);
            telemetry.addData("Position", position);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        setDrivePower(0, 0, 0);
    }

    /**
     * @param position
     * @param distance
     * @param acceleration
     * @return
     */
    public double powerRamp(double position, double distance, double acceleration) {

        position += 0.01;           // Necessary otherwise we're stuck at position 0 (sqrt(0) = 0)
        double normFactor = 1 / Math.sqrt(0.1 * distance);

        // Modeling a piece wise of power as a function of distance
        double p1 = normFactor * Math.sqrt(acceleration * position);
        double p2 = 1;
        double p3 = normFactor * Math.sqrt(acceleration * (distance - position));

        return Math.min(Math.min(p1, p2), p3);
    }


    /**
     * @param targetAngle
     * @param MOE
     */
    public void turn(double targetAngle, double MOE) {
        System.out.println("Turning to " + targetAngle + " degrees");

        double currentAngle = imu.getAngle();
        double deltaAngle = Math.abs(targetAngle - currentAngle);
        double power = (targetAngle > currentAngle) ? 1 : -1;


        // Retrieve angle and MOE
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        while ((lowerBound >= currentAngle || currentAngle >= upperBound) && opModeIsActive()) {

            // Power Ramping based off a logistic piecewise
            double currentDeltaAngle = targetAngle - currentAngle;
            double anglePosition = deltaAngle - currentDeltaAngle + 0.01; // Added the 0.01 so that it doesn't get stuck at 0


            // Modeling a piece wise of power as a function of distance
            power = powerRamp(anglePosition, deltaAngle, 0.3);
            telemetry.addData("Power", power);
            telemetry.update();

            // Handle clockwise (+) and counterclockwise (-) motion
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
            //System.out.println("Power: " + power);

            currentAngle = imu.getAngle();


            telemetry.addData("IMU", imu.getAngle());
        }

        // Stop power
        setAllPower(0);
    }

    /**
     * @return average encoder position
     */
    double getPosition() {
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4.0;
    }


    /**
     * @param power
     */
    private void setAllPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * @param drive
     * @param strafe
     * @param turn
     */
    public void setDrivePower(double drive, double strafe, double turn) {
        fr.setPower(drive - strafe - turn);
        fl.setPower(drive + strafe + turn);
        br.setPower(drive + strafe - turn);
        bl.setPower(drive - strafe + turn);
    }

    /**
     * Resets Motors
     */
    private void resetMotors() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}