package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class PowerCellSearchChallenge extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_swerveDrive;
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private double powerCells = 0; // How many power cells we've picked up
    /**
     * Creates a new ExampleCommand.
     *
     * @param RobotContainer.m_shooter The subsystem used by this command.
     */
    public PowerCellSearchChallenge(SwerveDrive swerveDrive, Shooter shooter, Indexer indexer, Intake intake) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_swerveDrive = swerveDrive;
      m_shooter = shooter;
      m_indexer = indexer;
      m_intake = intake;
      addRequirements(shooter);
      addRequirements(indexer);
      addRequirements(intake);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intake.setIntakePercentOutput(0);
      m_indexer.setIndexerOutput(0);
      m_indexer.setKickerOutput(0);
      m_shooter.setPower(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (false);
    }
  }
