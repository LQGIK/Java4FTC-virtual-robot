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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import net.waring.java4ftc.ops.Claw;
import net.waring.java4ftc.ops.IMU;
import net.waring.java4ftc.utilities.Utils;
import net.waring.java4ftc.ops.SyncTask;


@Autonomous(name="Arm-Mecanum Encoder", group="Autonomous Linear Opmode")
//@Disabled
public class ArmMecanumAutoEncoder extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private IMU imu;
    private Claw claw;


    public void initialize(){
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

        // Claw (Custom wrapper class to handle the arm <DcMotor> and the hand <Servo>
        claw = new Claw("hand_servo", "arm_motor");
    }


    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        // Auto Instructions
        System.out.println("Turn 90");
        turn(90, 0.5);

        System.out.println("Turn -90");
        turn(-90, 0.5);


        // Testing out anonymous class passing
        System.out.println("Drive 1500");
        drive(3000, 1, new SyncTask() {
            @Override
            public void execute() {
                if (claw.getArmPosition() < 50);
                claw.setArmPower(0.5);
                claw.setHandPosition(Math.random());
            }
        });

        System.out.println("Turn -170");
        turn(-170, 0.5);

        /*System.out.println("Adjusting forward 300");
        driveStraight(300, 1);

        System.out.println("Turn -180");
        pointTurn(-180, 0.5, false);

        System.out.println("Turn 10");
        pointTurn(10, 0.5, false);

        System.out.println("Drive 300");
        driveStraight(300, 1);

        System.out.println("Turn 45");
        pointTurn(45, 0.5, false);

        System.out.println("Drive 1500");
        driveStraight(3000, 1);

        System.out.println("Turn -45");
        pointTurn(-45, 0.5, false);

        System.out.println("Turn 0");
        pointTurn(0, 0.5, false);*/

    }

    /**
     * @param targetAngle
     * @param MOE
     */
    private void turn(double targetAngle, double MOE) {

        // Power Decay
        double power = 1;
        double minPower = 0.2;

        double currentAngle = imu.getAngle();
        double deltaAngle = Math.abs(targetAngle - currentAngle);

        // Retrieve angle and MOE
        if (targetAngle < currentAngle) power = -power;
        double upperBound = targetAngle + MOE;
        double lowerBound = targetAngle - MOE;
        while (lowerBound >= currentAngle || currentAngle >= upperBound){

            // Power Ramping based off a sin method
            double rampedPower = 0.5 * Math.sin(Math.PI * Math.abs(currentAngle) / deltaAngle);
            power = (power * rampedPower) + minPower;


            // Handle clockwise (+) and counterclockwise (-) motion
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
            System.out.println("Power: " + power);

            currentAngle = imu.getAngle();
        }
        // Stop power
        setAllPower(0);
    }





    /**
     * Drive straight with PPID Controller
     * @param ticks
     * @param power
     */
    private void drive(int ticks, double power, SyncTask task) {

        // Reset encoders
        resetMotors();

        // Initialize power settings
        double leftPower = power;
        double rightPower = power;
        double learning_rate = 0.000001;

        fl.setPower(leftPower);
        fr.setPower(rightPower);
        bl.setPower(leftPower);
        br.setPower(rightPower);

        // PID Controller
        double startHeading = imu.getAngle();
        double position = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
        while(position < ticks){

            // Synchronous Task Execution
            if (task != null){
                task.execute();
            }


            // Power Ramping based off a sin method
            double rampedPower = Math.sin(Math.PI * position / ticks);
            leftPower = (leftPower * rampedPower) + 0.5; rightPower = (rightPower * rampedPower) + 0.5;


            // Direction Tuning [ PID Controller ]
            double currentHeading = imu.getAngle();
            double error = startHeading - currentHeading;
            double correction = error * learning_rate;
            leftPower -= correction;
            rightPower += correction;


            //System.out.println("IMU: " + currentHeading);
            //System.out.println("Correction: " + correction);
            //System.out.println("Right Power: " + rightPower + ", Left Power: " + leftPower);

            fl.setPower(leftPower);
            bl.setPower(leftPower);
            fr.setPower(rightPower);
            br.setPower(rightPower);

            position = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;
        }

        // Stop motors
        setAllPower(0);


    }

    /**
     * @param power
     */
    private void setAllPower(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * Resets Motors
     */
    private void resetMotors(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Maps value to a given range
     * @param value
     * @param min
     * @param max
     * @return
     */
    private double map(double value, double min, double max){
        return (value - min) / (max - min);
    }
}
