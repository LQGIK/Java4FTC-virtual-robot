package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.GamePad;

public class Controller extends GamePad{

    public Thumbstick getRightThumbstick() {
        return new Controller.Thumbstick(this.right_stick_x, this.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Controller.Thumbstick(this.left_stick_x, this.left_stick_y);
    }

   public class Thumbstick {

        private double rawX;
        private double rawY;
        private double shiftedX;
        private double shiftedY;

        public Thumbstick(Double x, Double y) {
            this.rawX = x;
            this.rawY = y;
        }

        public Thumbstick(Float x, Float y) {
            this.rawX = x;
            this.rawY = y;
        }

        public double getX() {
            return rawX;
        }

        public double getY() {
            return rawY;
        }

        public void setShift(double shiftAngle) {
            this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
            this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedX() {
            return shiftedX;
        }

        public double getShiftedY() {
            return shiftedY;
        }

        public double getShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getInvertedX() {
            return -rawX;
        }

        public double getInvertedY() {
            return -rawY;
        }

        public double getInvertedShiftedX() {
            return -shiftedX;
        }

        public double getInvertedShiftedY() {
            return -shiftedY;
        }

        public double getInvertedShiftedX(Double shiftAngle) {
            return -this.getShiftedY();
        }

        public double getInvertedShiftedY(Double shiftAngle) {
            return -this.getShiftedX();
        }
    }

   public boolean DPADPress() {return dpad_down || dpad_left || dpad_right || dpad_up;}

}
