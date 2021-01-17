package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickWrapper extends Joystick {

    private boolean[] invertedAxis;
    private double[] deadbands;
    public JoystickWrapper(int port) {
        super(port);
        // TODO: Fix this to use a variable based on number of raw axis. this.getAxisCount() crashes the code
        invertedAxis = new boolean[10];
        deadbands = new double[10];
    }

    public void invertRawAxis(int axis, boolean inverted) {
        invertedAxis[axis] = inverted;
    }

    public void setAxisDeadband(int axis, double deadband) {
        this.deadbands[axis] = deadband;
    }

    @Override
    public double getRawAxis(int axis) {
        return Math.abs(super.getRawAxis(axis)) > deadbands[axis] ?
                (invertedAxis[axis] ? -super.getRawAxis(axis) : super.getRawAxis(axis)) : 0;
    }
}
