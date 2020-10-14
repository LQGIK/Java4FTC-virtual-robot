package net.waring.java4ftc.ops;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import virtual_robot.util.AngleUtils;

/**
 * A class that handles raw encoder values from odometer wheels and returs x,y,heading robot positioning.
 * This particular implementation was based on 8397's original example code:
 * https://github.com/Beta8397/virtual_robot/blob/master/TeamCode/src/org/firstinspires/ftc/teamcode/EncBot.java
 * (we removed all the nasty abstraction with arrays and stuff)
 * @author Alex Appleby, team 16896
 */
public class OdometrySystem {

    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0;
    private final double ENCODER_WIDTH = 12.0;

    /*Encoder classes must be DcMotorEx instead of DcMotor because of implementation in EncoderBot
      IRL DcMotor is perfectly sufficient for reading encoder vals. sim = bad.
      We could cast them but DcMotorEx is fine. It's just cause for confusion with newer teams who haven't seen DcMotorEx */
    private DcMotorEx rightOdo, leftOdo, xOdo; // (Odo is slang for odometer)

    /* in inches / degrees, with 8397's axis conventions.
       the class itself returns values with standard conventions (right x axis, up y axis, CCW-positive radians with zero on x axis)
       unit conversions are done in the getters for x, y, heading
       Always use standard conventions for your model when possible.
    */
    private double x, y, heading;

    private int prevRight, prevLeft, prevX;

    // initialization in constructor is almost always better than an init() method
    public OdometrySystem(HardwareMap hardwareMap) {
        rightOdo = hardwareMap.get(DcMotorEx.class, "enc_right");
        leftOdo = hardwareMap.get(DcMotorEx.class, "enc_left");
        xOdo = hardwareMap.get(DcMotorEx.class, "enc_x");

        prevRight = rightOdo.getCurrentPosition();
        prevLeft = leftOdo.getCurrentPosition();
        prevX = xOdo.getTargetPosition();
    }

    /**
     * call this method with every loop of your OpMode; this code should be run iteratively as fast as possible.
     * In a different application, you'd ideally want to put something like this on it's own Thread, but the FTC SDK isn't
     * designed for multithreaded hardware reads and writes. You could probably pull multithreaded off with the simulator, but we
     * did it this way to limit abstraction from proper FTC code.
     */
    public void update(){

        int rightTicks = rightOdo.getCurrentPosition();
        int leftTicks = leftOdo.getCurrentPosition();
        int xTicks = xOdo.getCurrentPosition();

        int newRightTicks = rightTicks - prevRight;
        int newLeftTicks = leftTicks - prevLeft;
        int newXTicks = xTicks - prevX;

        prevRight = rightTicks;
        prevLeft = leftTicks;
        prevX = xTicks;

        //###### 8397 code, could be dubious if run on an actual robot. tread carefully. ######
        double rightDist = newRightTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double leftDist = -newLeftTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double dyR = 0.5 * (rightDist + leftDist);
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
        double dxR = -newXTicks * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double avgHeadingRadians = heading + headingChangeRadians / 2.0;
        double cos = Math.cos(avgHeadingRadians);
        double sin = Math.sin(avgHeadingRadians);
        x += dxR*sin + dyR*cos;
        y += -dxR*cos + dyR*sin;
        heading = AngleUtils.normalizeRadians(heading + headingChangeRadians);
        //###### 8397 code end ######
    }

    public double getX(){
        return -y; //convert to standard coordinate conventions
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public double getY(){
        return x; //convert to standard coordinate conventions
    }

    public double getHeading(){
            return heading + (Math.PI / 2.0); //convert to standard coordinate conventions
    }
}
