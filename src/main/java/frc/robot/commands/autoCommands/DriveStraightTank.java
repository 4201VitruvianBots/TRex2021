package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class DriveStraightTank extends SequentialCommandGroup {
    public DriveStraightTank(SwerveDrive swerveDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        // new Translation2d(1, 0),
                        // new Translation2d(2, 0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config
        );

        RamseteCommand driveStraight = new RamseteCommand(
            exampleTrajectory,
            swerveDrive::getPose,
            new RamseteController(),
            Constants.DriveConstants.kTankKinematics,
            swerveDrive::setTankSpeeds,
            swerveDrive
        );

        addCommands(new ResetOdometry(swerveDrive).andThen(()-> swerveDrive.setTankDirection(new Rotation2d())),
                driveStraight.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));// Run path following command, then stop at the end.
    }
}
