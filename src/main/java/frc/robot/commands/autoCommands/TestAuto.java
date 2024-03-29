package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(SwerveDrive swerveDrive, FieldSim fieldSim) {
        int[][] waypointsRaw = { // {x, y, rot}
                {30,30,0},
                {90,30,45},
                {90,90,90}
        };
        Pose2d[] waypoints = new Pose2d[waypointsRaw.length];
        for (int i = 0; i < waypointsRaw.length; i++) {
                waypoints[i] = new Pose2d(Units.inchesToMeters(waypointsRaw[i][0]), Units.inchesToMeters(waypointsRaw[i][1]), new Rotation2d(Units.degreesToRadians(waypointsRaw[i][2])));
        }
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
        // ArrayList<Pose2d> trajectoryStates = new ArrayList<Pose2d>();
        //         trajectoryStates.addAll(trajectory.getStates().stream()
        //                 .map(state -> state.poseMeters)
        //                 .collect(Collectors.toList()));

        // fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);

        addCommands(new WaitCommand(0).andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));// Run path following command, then stop at the end.
    }
}
