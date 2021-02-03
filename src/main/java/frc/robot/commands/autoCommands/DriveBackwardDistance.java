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
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class DriveBackwardDistance extends SequentialCommandGroup {
        public DriveBackwardDistance(SwerveDrive swerveDrive, double distance) { // Distance in meters
                Pose2d startPosition = new Pose2d();
                Pose2d endPosition = new Pose2d(-distance, 0, new Rotation2d());
                TrajectoryConfig configA = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.DriveConstants.kDriveKinematics);
                configA.setReversed(true);
                configA.setEndVelocity(0);
                ArrayList<Pose2d> driveForwardPath = new ArrayList();
                driveForwardPath.add(startPosition);
                driveForwardPath.add(endPosition);

                SwerveControllerCommand driveForwardCommand = new SwerveControllerCommand(
                exampleTrajectory,
                swerveDrive::getPose, //Functional interface to feed supplier
                Constants.DriveConstants.kDriveKinematics,

                //Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),

                swerveDrive::setModuleStates,

                swerveDrive

        );
        
                addCommands(
                        new SetOdometry(driveTrain, startPosition),
                        //new SetDriveNeutralMode(driveTrain,0),
                        driveForwardCommand
                    );
        }
        
}
