package frc.vitruvianlib.utils;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class RevServo extends PWM {
    private static final double kMaxServoAngle = 270;
    private static final double kMinServoAngle = 0;

    protected static final double kDefaultMaxServoPWM = 2.5;
    protected static final double kDefaultMinServoPWM = 0.5;

    /**
     * Constructor.<br>
     *
     * <p>By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br>
     * By default {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel The PWM channel to which the servo is attached. 0-9 are on-board, 10-19 are on
     *     the MXP port
     */
    public RevServo(final int channel) {
        super(channel);
        setBounds(kDefaultMaxServoPWM, 0, 0, 0, kDefaultMinServoPWM);
        setPeriodMultiplier(PeriodMultiplier.k4X);

        HAL.report(FRCNetComm.tResourceType.kResourceType_Servo, getChannel() + 1);
        SendableRegistry.setName(this, "RevServo", getChannel());
    }

    /**
     * Set the servo position.
     *
     * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     *
     * @param value Position from 0.0 to 1.0.
     */
    public void set(double value) {
        setPosition(value);
    }

    /**
     * Get the servo position.
     *
     * <p>Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
     *
     * @return Position from 0.0 to 1.0.
     */
    public double get() {
        return getPosition();
    }

    /**
     * Set the servo angle.
     *
     * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
     * test).
     *
     * <p>Servo angles that are out of the supported range of the servo simply "saturate" in that
     * direction In other words, if the servo has a range of (X degrees to Y degrees) than angles of
     * less than X result in an angle of X being set and angles of more than Y degrees result in an
     * angle of Y being set.
     *
     * @param degrees The angle in degrees to set the servo.
     */
    public void setAngle(double degrees) {
        if (degrees < kMinServoAngle) {
            degrees = kMinServoAngle;
        } else if (degrees > kMaxServoAngle) {
            degrees = kMaxServoAngle;
        }

        setPosition(((degrees - kMinServoAngle)) / getServoAngleRange());
    }

    /**
     * Get the servo angle.
     *
     * <p>Assume that the servo angle is linear with respect to the PWM value (big assumption, need to
     * test).
     *
     * @return The angle in degrees to which the servo is set.
     */
    public double getAngle() {
        return getPosition() * getServoAngleRange() + kMinServoAngle;
    }

    private double getServoAngleRange() {
        return kMaxServoAngle - kMinServoAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RevServo");
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}
