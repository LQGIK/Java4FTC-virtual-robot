package net.waring.java4ftc.ops;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import net.waring.java4ftc.utilities.Utils;

public class Claw {

    private Servo hand;
    private DcMotor arm;

    public Claw(String clawName, String armName){
        HardwareMap hwMap = Utils.getHardwareMap();
        hand = hwMap.get(Servo.class, "hand_servo");
        arm = hwMap.get(DcMotor.class, "arm_motor");

        hand.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setHandPosition(double position){
        hand.setPosition(position);
    }

    public double getHandPosition(){
        return hand.getPosition();
    }

    public void setArmPower(double power){
        arm.setPower(power);
    }

    public double getArmPosition(){
        return arm.getCurrentPosition();
    }

}
