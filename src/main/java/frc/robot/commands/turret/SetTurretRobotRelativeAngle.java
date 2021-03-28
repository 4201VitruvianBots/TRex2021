package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Creating the turret subsystem, timer, and setpoint value.
 */
public class SetTurretRobotRelativeAngle extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final double m_setpoint;
    private double startTime;

    /**
     * Creating commands.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetTurretRobotRelativeAngle(Turret subsystem, double setpoint) {
        m_turret = subsystem;
        m_setpoint = setpoint;
        /**
         * adds specified requirements to the turret subsystem
         */
        addRequirements(subsystem);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        m_turret.setRobotCentricSetpoint(m_setpoint);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(m_turret.getTurretAngle() - m_setpoint) < 1;
    }
}
