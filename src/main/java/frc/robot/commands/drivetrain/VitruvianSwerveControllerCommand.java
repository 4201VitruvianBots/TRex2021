package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class VitruvianSwerveControllerCommand extends SwerveControllerCommand {
    private static Timer trajectoryTimer = new Timer();
    private Trajectory m_trajectory;
    private SwerveDrive m_driveTrain;
    private Pose2d finalPose;

    /** This is a wrapper class of @class SwerveControllerCommand. I'm using this to get where the robot ideally should
     *  be for debugging/testing purposes.
     *
     */
    public VitruvianSwerveControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            SwerveDrive swerveDrive) {
        this(
                trajectory,
                pose,
                kinematics,
                xController,
                yController,
                thetaController,
                () ->
                        trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
                outputModuleStates,
                swerveDrive);
    }

    public VitruvianSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics, PIDController xController, PIDController yController, ProfiledPIDController thetaController, Supplier<Rotation2d> desiredRotation, Consumer<SwerveModuleState[]> outputModuleStates, SwerveDrive swerveDrive) {
        super(trajectory, pose, kinematics, xController, yController, thetaController, desiredRotation, outputModuleStates, swerveDrive);
        m_trajectory = trajectory;
        m_driveTrain = swerveDrive;
        finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
    }

    @Override
    public void initialize() {
        super.initialize();
        trajectoryTimer.reset();
        trajectoryTimer.start();
        m_driveTrain.setCurrentTrajectory(m_trajectory);
    }

    @Override
    public void execute() {
        super.execute();
        m_driveTrain.setTrajectoryTime(trajectoryTimer.get());

    }

    // Don't do this, if robot pose is faster than sim pose, then you're just going to end up stuck on the next pose
//    @Override
//    public boolean isFinished() {
//        var robotPose = m_driveTrain.getPose();
//        double deltaX = Units.metersToFeet(Math.abs(robotPose.getX() - finalPose.getX()));
//        double deltaY = Units.metersToFeet(Math.abs(robotPose.getY() - finalPose.getY()));
////        double deltaRot = Math.abs(m_pose.get().getRotation().getDegrees() - m_path.get(m_path.size() - 1).getRotation().getDegrees());
//        boolean isFinished = (deltaX < Units.feetToMeters(0.25)) && (deltaY < Units.feetToMeters(0.25));
//        return isFinished;
//    }
}
