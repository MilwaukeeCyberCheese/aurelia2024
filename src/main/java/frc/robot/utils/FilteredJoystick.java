package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class FilteredJoystick {
    private Joystick joystick;

    /**
     * Filter class for the joysticks
     * 
     * @param port the port the joystick is plugged into
     */
    public FilteredJoystick(int port) {
        joystick = new Joystick(port);
    }

    /**
     * Returns the x-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getX(double deadzone) {
        return MathUtil.applyDeadband(joystick.getX(), deadzone);
    }

     /**
     * Returns the y-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getY(double deadzone) {
        return MathUtil.applyDeadband(joystick.getY(), deadzone) * -1;
    }

     /**
     * Returns the z-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getZ(double deadzone) {
        return MathUtil.applyDeadband(joystick.getZ(), deadzone);
    }

     /**
     * Returns the throttle-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getThrottle(double deadzone) {
        return ((MathUtil.applyDeadband(joystick.getThrottle(), deadzone) * -1) + 1) / 2;
    }

     /**
     * Returns the twist-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getTwist(double deadzone) {
        return MathUtil.applyDeadband(joystick.getTwist(), deadzone);
    }

     /**
     * Returns the x-value of the joystick
     * 
     */
    public double getX() {
        return MathUtil.applyDeadband(joystick.getX(), Constants.OIConstants.kDriveDeadband);
    }

     /**
     * Returns the y-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getY() {
        return MathUtil.applyDeadband(joystick.getY(), Constants.OIConstants.kDriveDeadband) * -1;
    }

     /**
     * Returns the z-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getZ() {
        return MathUtil.applyDeadband(joystick.getZ(), Constants.OIConstants.kDriveDeadband);
    }

     /**
     * Returns the throttle-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getThrottle() {
        return ((MathUtil.applyDeadband(joystick.getThrottle(), Constants.OIConstants.kDriveDeadband) * -1) + 1) / 2;
    }

     /**
     * Returns the twist-value of the joystick
     * 
     * @param deadzone zone in which no value is returned
     */
    public double getTwist() {
        return MathUtil.applyDeadband(joystick.getTwist(), 0.2);
    }

    /**
     * Returns whether or not the trigger is pressed
     * 
     * @return boolean
     */
    public boolean getTriggerActive() {
        return joystick.getRawButton(1);
    }

    /**
     * Returns if any POVButton is pressed or not
     * 
     * @return boolean
     */
    public boolean getPOVPressed() {
        return joystick.getPOV() != -1;
    }

    /**
     * Returns depending on which POVButton is pressed
     * 
     * @return int
     */
    public int getPOVButton() {
        int POVButton;
        if (joystick.getPOV() != -1) {
            switch (joystick.getPOV()) {
                case 0:
                    POVButton = 8;
                    break;

                case 45:
                    POVButton = 9;
                    break;

                case 90:
                    POVButton = 6;
                    break;

                case 135:
                    POVButton = 3;
                    break;

                case 180:
                    POVButton = 2;
                    break;

                case 225:
                    POVButton = 1;
                    break;

                case 270:
                    POVButton = 4;
                    break;

                case 315:
                    POVButton = 7;
                    break;

                case 360:
                    POVButton = 8;
                    break;
                default:
                    POVButton = 0;
            }
            return POVButton;
        } else {
            return 0;
        }
    }

     
    public boolean getButtonTwo() {
        return joystick.getRawButton(2);
    }

    public boolean getButtonThree() {
        return joystick.getRawButton(3);
    }

    public boolean getButtonFour() {
        return joystick.getRawButton(4);
    }

    public boolean getButtonFive() {
        return joystick.getRawButton(5);
    }

    public boolean getButtonSix() {
        return joystick.getRawButton(6);
    }

    public boolean getButtonSeven() {
        return joystick.getRawButton(7);
    }

    public boolean getButtonEight() {
        return joystick.getRawButton(8);
    }

    public boolean getButtonNine() {
        return joystick.getRawButton(9);
    }

    public boolean getButtonTen() {
        return joystick.getRawButton(10);
    }

    public boolean getButtonEleven() {
        return joystick.getRawButton(11);
    }

    public boolean getButtonTwelve() {
        return joystick.getRawButton(12);
    }
}