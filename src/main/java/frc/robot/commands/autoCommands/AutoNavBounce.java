package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavBounce extends SequentialCommandGroup {
    public AutoNavBounce(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypointsA = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
        };
        Pose2d[] waypointsB = {
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(105), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-250))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-240))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
        };
        Pose2d[] waypointsC = {
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
        };
        Pose2d[] waypointsD = {
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180)))
        };
        Pose2d startPosition = waypointsA[0];

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

        var trajectoryA = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsA.clone()), config);
        var trajectoryC = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsC.clone()), config);
        config.setReversed(true);
        var trajectoryB = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsB.clone()), config);
        var trajectoryD = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypointsD.clone()), config);

        var trajectoryStates = new ArrayList<Pose2d>();
        trajectoryStates.addAll(trajectoryA.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryB.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryC.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        trajectoryStates.addAll(trajectoryD.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryA, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryB, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryC, ()-> new Rotation2d()));
        addCommands(TrajectoryUtils.generateSwerveCommand(swerveDrive, trajectoryD, ()-> new Rotation2d()));

//        // Create config for trajectory
//        TrajectoryConfig config =
//                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                        // Add kinematics to ensure max speed is actually obeyed
//                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
//        config.setReversed(false);
//
//        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
//                new SetDriveNeutralMode(swerveDrive, true)
//        );
//
//        double[] startVelocities = {config.getMaxVelocity(), 0, config.getMaxVelocity(), config.getMaxVelocity(), 0,
//                config.getMaxVelocity(), config.getMaxVelocity(), 0, 0};
//        double[] endVelocities = {0, config.getMaxVelocity(), config.getMaxVelocity(), 0, config.getMaxVelocity(),
//                config.getMaxVelocity(), 0, 0};
//
//        var trajectoryStates = new ArrayList<Pose2d>();
//        for (int i = 0; i < waypoints.length - 1; i++) {
//                config.setStartVelocity(startVelocities[i]);
//                config.setEndVelocity(endVelocities[i]);
//                config.setReversed(pathIsReversed[i]);
//                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
//                        List.of(),
//                        waypoints[i + 1],
//                        config);
//
//                trajectoryStates.addAll(trajectory.getStates().stream()
//                        .map(state -> state.poseMeters)
//                        .collect(Collectors.toList()));
//
//                SwerveControllerCommand command = new SwerveControllerCommand(
//                        trajectory,
//                        swerveDrive::getPose,
//                        Constants.DriveConstants.kDriveKinematics,
//                        //Position controllers
//                        new PIDController(Constants.AutoConstants.kPXController, 0,0),
//                        new PIDController(Constants.AutoConstants.kPYController, 0,0),
//                        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
//                        Constants.AutoConstants.kThetaControllerConstraints),
//                        () -> new Rotation2d(0),    // Fixes auto
//                        swerveDrive::setModuleStates,
//
//                        swerveDrive
//                );
//                addCommands(command.alongWith(new InstantCommand(() -> swerveDrive.setCurrentTrajectory(trajectory))));
//        }
//        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));// Run path following command, then stop at the end.
    }
}
