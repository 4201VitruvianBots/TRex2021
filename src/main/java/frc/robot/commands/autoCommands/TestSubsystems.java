package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.indexer.SetCaroselOutput;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.shooter.SetUptakeOutput;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uptake;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class TestSubsystems extends SequentialCommandGroup {
    public TestSubsystems(Shooter shooter, Uptake uptake, Intake intake, Indexer indexer) {
        addCommands(
              new SetIntakePiston(intake, true),
              new AutoControlledIntake(intake, indexer).withTimeout(4),
              new SetIntakePiston(intake, false),
              new WaitCommand(2),
              new SetRpmSetpoint(shooter, 2000, false),
              new RapidFireSetpoint(shooter, indexer, uptake).withTimeout(5),
              new SetRpmSetpoint(shooter, 0, false)
        );
    }
}
