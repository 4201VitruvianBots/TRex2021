package frc.robot.subsystems;

public class MotorStallException extends IllegalStateException {
    public MotorStallException(String msg) {
        super(msg);
    }
}
