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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180)))
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

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints.clone()), config);

        ArrayList<Pose2d> trajectoryStates = new ArrayList<Pose2d>();
        trajectoryStates.addAll(trajectory.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));

        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectory, () -> new Rotation2d()));

//        for (int i = 0; i < waypoints.length - 1; i++) {
//                if (i != 0) {
//                        config.setEndVelocity(config.getMaxVelocity() + 99);
//                        config.setStartVelocity(config.getMaxVelocity() + 99);
//                }
//                if (i == waypoints.length - 2) {
//                        config.setEndVelocity(0);
//                }
//                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
//                        List.of(),
//                        waypoints[i + 1],
//                        config);
//
//                trajectoryStates.addAll(trajectory.getStates().stream()
//                                .map(state -> state.poseMeters)
//                                .collect(Collectors.toList()));
//
////                var command = TrajectoryUtils.generateRamseteCommand(swerveDrive, trajectory);
//                var command = TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectory, ()-> new Rotation2d());
//
//                addCommands(command);
//        }
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(new InstantCommand(() -> swerveDrive.drive(0, 0, 0, false, false), swerveDrive));// Run path following command, then stop at the end.
    }
}
