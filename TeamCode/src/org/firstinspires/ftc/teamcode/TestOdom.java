package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import net.waring.java4ftc.ops.OdometrySystem;
import org.firstinspires.ftc.teamcode.demos.EncBot;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders.
 *
 * EDITED by bfr to use the OdometrySystem
 */

//@Disabled
@TeleOp(name = "TestOdom", group = "OdomBot")
public class TestOdom extends LinearOpMode {

    EncBot bot = new EncBot();

    DcMotor fr, fl, br, bl;

    public void runOpMode(){
        gamepad1.setJoystickDeadzone(0.05f);
        bot.init(hardwareMap);
        OdometrySystem odom = new OdometrySystem(hardwareMap);


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

        waitForStart();
        while (opModeIsActive()){

            odom.update();

            double position = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4.0;

            double ticks = 5100;
            double angle = 67.5;

            double radians = angle * Math.PI / 180;
            double yFactor = Math.sin(radians);         // Unit Circle Y
            double xFactor = Math.cos(radians);         // Unit Circle X
            double y = yFactor * ticks;                 // Get the Y-Ticks (The ticks that our DcMotor wheels will reference bc they don't have an x-encoder
            double x = xFactor * ticks;                 // Get the X-Ticks (Don't actually need this)

            // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
            // If you take the lowest number and normalize it then our higher number will be hugeeeee
            double normalizeToPower = 1 / Math.max(xFactor, yFactor);
            double drive = normalizeToPower * yFactor;                 // Get the ratio of powers for the drive
            double strafe = normalizeToPower * xFactor;                // Get the ratio of powers for the strafe


            telemetry.addData("POSE", "x = %.1f  y = %.1f  h = %.01f", odom.getX(), odom.getY(), Math.toDegrees(odom.getHeading()));
            telemetry.addData("Diagonal ", Math.sqrt(Math.pow(odom.getX(), 2) + Math.pow(odom.getY(), 2)));
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Position", position);
//            telemetry.addData("Back Left", "T = %d  V = %.0f", bot.motors[0].getCurrentPosition(), bot.motors[0].getVelocity());
//            telemetry.addData("Front Left", "T = %d  V = %.0f", bot.motors[1].getCurrentPosition(), bot.motors[1].getVelocity());
//            telemetry.addData("Front Right", "T = %d  V = %.0f", bot.motors[2].getCurrentPosition(), bot.motors[2].getVelocity());
//            telemetry.addData("Back Right", "T = %d  V = %.0f", bot.motors[3].getCurrentPosition(), bot.motors[3].getVelocity());
//            telemetry.addData("Right Enc", "T = %d  V = %.0f", bot.encoders[0].getCurrentPosition(), bot.encoders[0].getVelocity());
//            telemetry.addData("Left Enc", "T = %d  V = %.0f", bot.encoders[1].getCurrentPosition(), bot.encoders[1].getVelocity());
//            telemetry.addData("X Enc", "T = %d  V = %.0f", bot.encoders[2].getCurrentPosition(), bot.encoders[2].getVelocity());
            telemetry.update();



            if (position < y){
                /*
                double px = gamepad1.left_stick_x;
                double py = -gamepad1.left_stick_y;
                double pa = gamepad1.left_trigger - gamepad1.right_trigger;
                bot.setDrivePower(px, py, pa);*/



                bot.setDrivePower(strafe, drive, 0);
            }
            else bot.setDrivePower(0, 0, 0);

        }
    }
}
