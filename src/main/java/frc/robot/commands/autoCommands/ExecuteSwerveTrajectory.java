package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.stream.Collectors;

public class ExecuteSwerveTrajectory extends SequentialCommandGroup {

    /**
     *
     * @param config trajectory config to use to generate trajectory
     * @param waypoints points (in inches and degrees) on trajectory
     * @return a trajectory through these points
     */
    public ExecuteSwerveTrajectory(SwerveDrive swerveDrive, TrajectoryConfig config, FieldSim fieldSim, int[]... waypoints) {
        Trajectory trajectory = TrajectoryUtils.generateSwerveTrajectory(config, waypoints);
        var trajectoryStates = new ArrayList<Pose2d>();
        trajectoryStates.addAll(trajectory.getStates().stream()
                .map(state -> state.poseMeters)
                .collect(Collectors.toList()));
        fieldSim.getField2d().getObject("trajectory").setPoses(trajectoryStates);
        SwerveControllerCommand command = new SwerveControllerCommand(
                trajectory,
                swerveDrive::getPose,
                Constants.DriveConstants.kDriveKinematics,
                //Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0,0),
                new PIDController(Constants.AutoConstants.kPYController, 0,0),
                new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints),
                swerveDrive::getHeadingToTarget,
                //() -> new Rotation2d(0),

                swerveDrive::setModuleStates,

                swerveDrive
        );
        addCommands(command);
    }
}