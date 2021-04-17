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

import java.awt.image.PackedColorModel;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class AutoNavBarrel extends SequentialCommandGroup {
    public AutoNavBarrel(SwerveDrive swerveDrive, FieldSim fieldSim) {
        int[][] waypointsRaw = {
                {40,90,0},
                {150,90,0},
                {176,45,-120},
                {135,45,120},
                {150,90,0},
                {250,96,30},
                {270,141,135},
                {210,120,-80},
                {290,45,-45},
                {330,45,45},
                {300,92,175},
                {180, 102, 180},
                {30,102,180}
        };
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(175), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(125), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(265), Units.inchesToMeters(135), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(165), new Rotation2d(Units.degreesToRadians(180))),
                new Pose2d(Units.inchesToMeters(215), Units.inchesToMeters(135), new Rotation2d(Units.degreesToRadians(-90))),
                new Pose2d(Units.inchesToMeters(285), Units.inchesToMeters(34), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180))),
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(-180))),
        
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
                new SetDriveNeutralMode(swerveDrive, true),
                new ExecuteSwerveTrajectory(swerveDrive, config, fieldSim, waypointsRaw)
        );

        /*var trajectoryStates = new ArrayList<Pose2d>();
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

                SwerveControllerCommand command = new SwerveControllerCommand(
                        trajectory,
                        swerveDrive::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        //Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0,0),
                        new PIDController(Constants.AutoConstants.kPYController, 0,0),
                        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),

                        swerveDrive::setModuleStates,

                        swerveDrive
                );
                addCommands(command);
        }
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);*/

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false)));// Run path following command, then stop at the end.
    }
}
