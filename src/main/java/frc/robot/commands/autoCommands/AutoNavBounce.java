package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavBounce extends SequentialCommandGroup {
    public AutoNavBounce(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
        };
        boolean[] pathIsReversed = {false, true, true, true, false, false, false, true};
        Pose2d startPosition = waypoints[0];

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
        config.setReversed(false);

        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
                new SetDriveNeutralMode(swerveDrive, true)
        );

        double[] startVelocities = {config.getMaxVelocity(), 0, config.getMaxVelocity(), config.getMaxVelocity(), 0, 
                config.getMaxVelocity(), config.getMaxVelocity(), 0, 0};
        double[] endVelocities = {0, config.getMaxVelocity(), config.getMaxVelocity(), 0, config.getMaxVelocity(), 
                config.getMaxVelocity(), 0, 0};

        for (int i = 0; i < waypoints.length - 1; i++) {
                config.setStartVelocity(startVelocities[i]);
                config.setEndVelocity(endVelocities[i]);
                config.setReversed(pathIsReversed[i]);
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                        List.of(),
                        waypoints[i + 1],
                        config);
                //var command = TrajectoryUtils.generateRamseteCommand(swerveDrive, trajectory);
                SwerveControllerCommand command = new SwerveControllerCommand(
                        trajectory,
                        swerveDrive::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        //Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
                        new PIDController(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController),
                        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),

                        swerveDrive::setModuleStates,

                        swerveDrive
                );
                addCommands(command);
        }

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false)));// Run path following command, then stop at the end.
    }
}
