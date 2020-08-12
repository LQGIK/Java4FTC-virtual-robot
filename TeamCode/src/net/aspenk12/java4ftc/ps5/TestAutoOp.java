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
 *//*


package net.aspenk12.java4ftc.team.opModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

*/
/**
 * Demonstrates empty OpMode
 *//*

@TeleOp(name = "Concept: TestTeleOp")
//@Disabled
public class TestAutoOp extends OpMode {

  private BNO055IMU imu;

  private DcMotor frontRightMotor;
  private DcMotor frontLeftMotor;
  private DcMotor backRightMotor;
  private DcMotor backLeftMotor;

  private ElapsedTime runtime = new ElapsedTime();



  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    // IMU Initialization
    BNO055IMU.Parameters params = new BNO055IMU.Parameters(); // Figure out what this does
    imu.initialize(params);
    // Parameters is a public innerclass of the IMU,


    // Motors
    frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
    backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");

    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


  }

  */
/*
 * Code to run when the op mode is first enabled goes here
 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
 *//*

  @Override
  public void init_loop() {

  }

  */
/*
 * This method will be called ONCE when start is pressed
 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
 *//*

  @Override
  public void start() {
    runtime.reset();
  }

  */
/*
 * This method will be called repeatedly in a loop
 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
 *//*

  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    double strafe = gamepad1.right_stick_x;
    double drive = gamepad1.right_stick_y;
    double turn = gamepad1.left_stick_x * -1;

    frontLeftMotor.setPower(drive - strafe + turn);
    frontRightMotor.setPower(drive + strafe - turn);
    backLeftMotor.setPower(drive + strafe + turn);
    backRightMotor.setPower(drive - strafe - turn);

    telemetry.addData("Strafe", strafe);
    telemetry.addData("Drive", drive);
    telemetry.addData("Turn", turn);
  }
}
*/
