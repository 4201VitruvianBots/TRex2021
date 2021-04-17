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
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavSlalom extends SequentialCommandGroup {
    public AutoNavSlalom(SwerveDrive swerveDrive, FieldSim fieldSim) {
        int[][] waypointsRaw = {
                /*{40,30,0},
                // {90,60,60},
                {120,90,0},
                {239,96,0},
                {266,60,-70},
//                {286,50,-45},
                {296,30,0},
                {350,60,90},
                {320,80,180},
                {285,60,-105},
                {245,16,180},
//                {200,15,180},
                {140,16,180},
                {105,60,105},
                {30,80,168}*/
                {30,30,0},
                {120,90,0},
                {180,90,0},
                {240,90,0},
                {270,60,0},     // That's hilarious
                {315,34,0},
                {315,86,0},
                {270,60,0},
                {240,30,0},
                {180,30,0},
                {120,30,0},
                {30,90,0}
        };
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(225), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(150))),
                new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(150)))
        };
        Pose2d startPosition = waypoints[0];

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond * 1.2,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared * 1.2)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
        config.setReversed(false);


        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
                new SetDriveNeutralMode(swerveDrive, true),
                // new InstantCommand(() -> swerveDrive.setHeadingToTargetPosition(new Pose2d(4.5, 4.5, new Rotation2d())), swerveDrive),
                new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypointsRaw)
        );

        /*Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[0],
                        List.of(waypoints[1].getTranslation(), 
                        waypoints[2].getTranslation(), 
                        waypoints[3].getTranslation(), 
                        waypoints[4].getTranslation(), 
                        waypoints[5].getTranslation(),
                        waypoints[6].getTranslation(),
                        waypoints[7].getTranslation(), 
                        waypoints[8].getTranslation()),
                        waypoints[9],
                        config);

        trajectoryStates.addAll(trajectory.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        SwerveControllerCommand command = new SwerveControllerCommand(
                        trajectory,
                        swerveDrive::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        //Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0,0),
                        new PIDController(Constants.AutoConstants.kPYController, 0,0),
                        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                            Constants.AutoConstants.kThetaControllerConstraints),
                        () -> new Rotation2d(0),    // Fixes auto
//                        swerveDrive::getHeadingToTarget,

                        swerveDrive::setModuleStates,

                        swerveDrive
                );
                addCommands(command);
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

                trajectoryStates.addAll(trajectory.getStates().stream()
                                .map(state -> state.poseMeters)
                                .collect(Collectors.toList()));

//                var command = TrajectoryUtils.generateRamseteCommand(swerveDrive, trajectory);
                SwerveControllerCommand command = new SwerveControllerCommand(
                        trajectory,
                        swerveDrive::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        //Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0,0),
                        new PIDController(Constants.AutoConstants.kPYController, 0,0),
                        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                            Constants.AutoConstants.kThetaControllerConstraints),
                        () -> new Rotation2d(0),    // Fixes auto
//                        swerveDrive::getHeadingToTarget,

                        swerveDrive::setModuleStates,

                        swerveDrive
                );
                addCommands(command);
        }*/
        //fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(new InstantCommand(() -> swerveDrive.drive(0, 0, 0, false), swerveDrive));// Run path following command, then stop at the end.
    }
}
