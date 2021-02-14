package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class SetOdometry extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_swerveDrive;
    private final FieldSim m_fieldSim;
    private Pose2d m_pose2d;
    /**
     * Creates a new ExampleCommand.
     *
     * @param swerveDrive The subsystem used by this command.
     */
    public SetOdometry(SwerveDrive swerveDrive, Pose2d pose2d) {
        this(swerveDrive, null, pose2d);
    }

    public SetOdometry(SwerveDrive swerveDrive, FieldSim fieldSim, Pose2d pose2d) {
        if(RobotBase.isSimulation() && fieldSim == null)
            System.out.println("SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

        m_swerveDrive = swerveDrive;
        m_fieldSim = fieldSim;
        m_pose2d = pose2d;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_swerveDrive.resetOdometry(m_pose2d, m_pose2d.getRotation());
        if (RobotBase.isSimulation()) {
            m_fieldSim.resetRobotPose(m_pose2d);
        }
        m_swerveDrive.resetEncoders();
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
