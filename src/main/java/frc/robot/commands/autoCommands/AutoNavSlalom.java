package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.VitruvianSwerveControllerCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(315), Units.inchesToMeters(86), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
        };
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

        for (int i = 0; i < waypoints.length - 1; i++) {
                if (i != 0) {
                        config.setEndVelocity(config.getMaxVelocity());
                        config.setStartVelocity(config.getMaxVelocity());
                }
                if (i == waypoints.length - 2) {
                        config.setEndVelocity(0);
                }
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                        List.of(),
                        waypoints[i + 1],
                        config);
                //var command = TrajectoryUtils.generateRamseteCommand(swerveDrive, trajectory);
                VitruvianSwerveControllerCommand command = new VitruvianSwerveControllerCommand(
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
