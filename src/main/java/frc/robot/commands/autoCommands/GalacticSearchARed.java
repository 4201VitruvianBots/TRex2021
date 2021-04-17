package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.List;

public class GalacticSearchARed extends SequentialCommandGroup {
    private final double percentageOfMaxVel = 0.3; // How much of max velocity to be going at when we intake a power cell

    public GalacticSearchARed(SwerveDrive swerveDrive, FieldSim fieldSim) {
        Pose2d[] waypoints = {
                new Pose2d(Units.inchesToMeters(15), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(90), new Rotation2d(Units.degreesToRadians(0))),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-60))),
                new Pose2d(Units.inchesToMeters(180), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(60))),
                new Pose2d(Units.inchesToMeters(345), Units.inchesToMeters(150), new Rotation2d(Units.degreesToRadians(60)))
        };
        Pose2d startPosition = waypoints[0];


        TrajectoryConfig config =
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);
        config.setReversed(false);

        addCommands(new SetOdometry(swerveDrive, fieldSim, startPosition),
                new SetDriveNeutralMode(swerveDrive, true)
        );

        for(int i = 0; i < waypoints.length - 1; i++) {
            if (i != 0) {
                config.setEndVelocity(config.getMaxVelocity() * percentageOfMaxVel);
                config.setStartVelocity(config.getMaxVelocity() * percentageOfMaxVel);
            }
            if (i == waypoints.length - 2) {
                config.setEndVelocity(0);
            }
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints[i],
                    List.of(),
                    waypoints[i + 1],
                    config);
            SwerveControllerCommand command = new SwerveControllerCommand(
                    trajectory,
                    swerveDrive::getPose,
                    Constants.DriveConstants.kDriveKinematics,
                    //Position controllers
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                            Constants.AutoConstants.kThetaControllerConstraints),

                    swerveDrive::setModuleStates,

                    swerveDrive
            );
            addCommands(command);
        }

        addCommands(new ResetOdometry(swerveDrive).andThen(() -> swerveDrive.drive(0, 0, 0, false)));// Run path following command, then stop at the end.
    }
}
