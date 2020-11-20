package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Controller;
import org.firstinspires.ftc.teamcode.Hardware.MecanumRobot;
import org.firstinspires.ftc.teamcode.Utilities.Utils;


@TeleOp(name = "Mecanum Drive", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    private MecanumRobot mecanumRobot;
    private Controller controller;


    // Toggle Variables
    private boolean absoluteControlMode;
    private boolean DPAD_Toggle;
    private boolean RB_Toggle;
    private boolean RBLastCycle;
    private boolean velocityToggle;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Utils.setHardwareMap(hardwareMap);
        Utils.setTelemetry(telemetry);
        mecanumRobot = new MecanumRobot();
        controller = new Controller();

        // Toggles
        absoluteControlMode = false;
        DPAD_Toggle = false;
        RB_Toggle = false;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // Get Thumbsticks
        Controller.Thumbstick rightThumbstick = controller.getRightThumbstick();
        Controller.Thumbstick leftThumbstick = controller.getLeftThumbstick();

        // Check Absolute Control Mode
        if (absoluteControlMode) rightThumbstick.setShift(mecanumRobot.imu.getAngle() % 360);
        else rightThumbstick.setShift(0);


        // ButtonTapped() compares current and previous states to handle single presses of buttons.
        // If we press RB once, toggle velocity shift
        RBLastCycle = Utils.buttonTapped(controller.right_bumper, RBLastCycle);
        if (RBLastCycle) velocityToggle = !velocityToggle;

       // Set Driver Values
        double drive = rightThumbstick.getInvertedShiftedY();
        double strafe = rightThumbstick.getInvertedShiftedX();
        double turn = leftThumbstick.getInvertedShiftedX();
        double velocity = (velocityToggle) ? 0.5 : 1;

        /*if (controller.right_bumper && !RBLastCycle ){
            velocityToggle = !velocityToggle; RBLastCycle = true;
        }
        else if (!controller.right_bumper) RBLastCycle = false;*/


        // DPAD Auto Turn
        if (controller.DPADPress()){
            if (controller.dpad_up) mecanumRobot.turn(MecanumRobot.Direction.NORTH, 1);
            else if (controller.dpad_right) mecanumRobot.turn(MecanumRobot.Direction.EAST, 1);
            else if (controller.dpad_left) mecanumRobot.turn(MecanumRobot.Direction.WEST, 1);
            else if (controller.dpad_down) mecanumRobot.turn(MecanumRobot.Direction.SOUTH, 1);
        }
        else mecanumRobot.setDrivePower(drive, strafe, turn, velocity);

        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        this.log();
    }

    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanumRobot.imu.getAngle());
        telemetry.addData("RGB", "(${mecanumRobot.colorSensor.red()}, ${mecanumRobot.colorSensor.green()}, ${mecanumRobot.colorSensor.blue()}");
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("Error", mecanumRobot.imu.getStartAngle() - mecanumRobot.imu.getAngle());
    }


    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {}

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {}
}


