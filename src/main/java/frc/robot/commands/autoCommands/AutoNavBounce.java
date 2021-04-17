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
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavBounce extends SequentialCommandGroup {
    public AutoNavBounce(SwerveDrive swerveDrive, FieldSim fieldSim) {
        int[][] waypointsRaw = {
                {30,90,0},
                {80,146,90},
                {115,85,100},
                {150, 37,180},
                {180,140,-90},
                {180, 80, -90},
                {260,30,30},
                {270,140,90},
                {310,90,135}
        };
        /*int[][] waypoints1 = {
                {30,90,0},
                {80,146,90},
        };
        int[][] waypoints2 = {
                {80,146,90},
                {115,85,100},
                {150, 37,180},
                {180,140,-90},
        };
        int[][] waypoints3 = {
                {180,140,-90},
                {180, 80, -90},
                {260,30,30},
                {270,140,90},
        };
        int[][] waypoints4 = {
                {270,140,90},
                {310,90,135}
        };*/
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(255), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(270), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180)))
        };
        boolean[] pathIsReversed = {false, true, true, false, false, false, true};
//        boolean[] pathIsReversed = {true, true, false, false, false, true};
        Pose2d startPosition = waypoints[0];

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
        config.setReversed(false);

        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
                new SetDriveNeutralMode(swerveDrive, true),
                new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypointsRaw)
                // new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypoints1),
                // new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypoints2),
                // new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypoints3),
                // new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypoints4),
        );

        /*double[] startVelocities = {config.getMaxVelocity(), 0, config.getMaxVelocity(), config.getMaxVelocity(), 0, 
                config.getMaxVelocity(), config.getMaxVelocity(), 0, 0};
        double[] endVelocities = {0, config.getMaxVelocity(), config.getMaxVelocity(), 0, config.getMaxVelocity(), 
                config.getMaxVelocity(), 0, 0};

        var trajectoryStates = new ArrayList<Pose2d>();
        for (int i = 0; i < waypoints.length - 1; i++) {
                config.setStartVelocity(startVelocities[i]);
                config.setEndVelocity(endVelocities[i]);
                config.setReversed(pathIsReversed[i]);
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                        List.of(),
                        waypoints[i + 1],
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
                        swerveDrive::setModuleStates,

                        swerveDrive
                );
                addCommands(command.alongWith(new InstantCommand(() -> swerveDrive.setCurrentTrajectory(trajectory))));
        }
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);*/

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false)));// Run path following command, then stop at the end.
    }
}
