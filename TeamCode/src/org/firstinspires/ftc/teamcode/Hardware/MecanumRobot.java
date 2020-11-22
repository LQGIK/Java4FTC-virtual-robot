package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.ColorSensorImpl;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Utilities.Utils.telemetry;

public class MecanumRobot implements Robot {

   private HardwareMap hardwareMap;
   private DcMotor fr, fl, br, bl;
   public ColorSensorImpl colorSensor;
   public ColorSensor colorSensorBase;
   public IMU imu;


   public MecanumRobot(){
      initRobot();
   }

   public enum Direction {
      NORTH,
      WEST,
      EAST,
      SOUTH
   }

   public void initRobot() {
      Utils.telemetry.addData("Status", "Initialized");

      // Init Motors
      fr = Utils.hardwareMap.get(DcMotor.class, "front_right_motor");
      fl = Utils.hardwareMap.get(DcMotor.class, "front_left_motor");
      br = Utils.hardwareMap.get(DcMotor.class, "back_right_motor");
      bl = Utils.hardwareMap.get(DcMotor.class, "back_left_motor");
      initMotors();

      // Sensors
      colorSensorBase = Utils.hardwareMap.get(ColorSensor.class, "color_sensor");
      colorSensor = new ColorSensorImpl(colorSensorBase);
      imu = new IMU("imu");
   }

   /**
    * (Re)Init Motors
    */
   public void initMotors(){
      fr.setDirection(DcMotorSimple.Direction.FORWARD);
      fl.setDirection(DcMotorSimple.Direction.REVERSE);
      br.setDirection(DcMotorSimple.Direction.FORWARD);
      bl.setDirection(DcMotorSimple.Direction.REVERSE);

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
    * @return average encoder position
    */
   public double getPosition(){
      return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4.0;
   }

   /**
    * @param power
    */
   @Override
   public void setAllPower(double power){
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
   public void setDrivePower(double drive, double strafe, double turn, double velocity) {
      fr.setPower((drive - strafe - turn) * velocity);
      fl.setPower((drive + strafe + turn) * velocity);
      br.setPower((drive + strafe - turn) * velocity);
      bl.setPower((drive - strafe + turn) * velocity);
   }

   public void turn(Direction direction, double MOE) {
      System.out.println("Turning to " + direction + " degrees");

      double targetAngle = 0;
      switch (direction) {
         case NORTH:
            targetAngle = 0;
            Utils.print("NORTH");
            break;
         case WEST:
            targetAngle = 90;
            Utils.print("WEST");
            break;
         case EAST:
            targetAngle = -90;
            Utils.print("EAST");
            break;
         case SOUTH:
            targetAngle = 180;
            Utils.print("SOUTH");
            break;
      }
      targetAngle += imu.getDeltaAngle();
      double currentAngle = imu.getAngle();
      double upperBound = targetAngle + MOE;
      double lowerBound = targetAngle - MOE;
      if (lowerBound >= currentAngle || currentAngle >= upperBound){
         double coTermAngle = Utils.coTerminal(targetAngle - currentAngle);
         double turn = (coTermAngle <= 0) ? 1 : -1;
         setDrivePower(0, 0, turn, 0.3);
      }
      telemetry.addData("TargetAngle", targetAngle);
   }

   /**
      * @param targetAngle
      * @param MOE
   */
   public void turn(double targetAngle, double MOE) {
         System.out.println("Turning to " + targetAngle + " degrees");

         double currentAngle = imu.getAngle();
         double deltaAngle = Math.abs(targetAngle - currentAngle);
         double power;
         double position = getPosition();


         // Retrieve angle and MOE
         double upperBound = targetAngle + MOE;
         double lowerBound = targetAngle - MOE;
         while (lowerBound >= currentAngle || currentAngle >= upperBound) {

            // Power Ramping based off a logistic piecewise
            double currentDeltaAngle = targetAngle - currentAngle;
            double anglePosition = deltaAngle - currentDeltaAngle + 0.01; // Added the 0.01 so that it doesn't get stuck at 0


            // Modeling a piece wise of power as a function of distance
            power = Utils.powerRamp(anglePosition, deltaAngle, 0.1);

            // Handle clockwise (+) and counterclockwise (-) motion
            setDrivePower(0, 0, 1, power);

            currentAngle = imu.getAngle();
         }

         // Stop power
         setAllPower(0);
      }
}