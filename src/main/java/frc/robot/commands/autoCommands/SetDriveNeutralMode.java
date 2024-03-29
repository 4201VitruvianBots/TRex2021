package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class SetDriveNeutralMode extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_swerveDrive;
    private final boolean m_mode;
    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDrive The subsystem used by this command.
     */
    public SetDriveNeutralMode(SwerveDrive swerveDrive, boolean mode) {
        m_swerveDrive = swerveDrive;
        m_mode = mode;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_swerveDrive.setDriveNeutralMode(m_mode);
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
