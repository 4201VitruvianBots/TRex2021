package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class SetModuleStates extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_swerveDrive;
    private final SwerveModuleState[] m_states;
    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDrive The subsystem used by this command.
     */
    public SetModuleStates(SwerveDrive swerveDrive, SwerveModuleState... states) {
        m_swerveDrive = swerveDrive;
        m_states = states;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_swerveDrive.setModuleStates(m_states);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
