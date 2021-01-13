package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class VitruvianRamseteCommand extends RamseteCommand {
    Trajectory m_trajectory;
    TrajectoryConfig m_config;
    ArrayList<Pose2d> m_path;
    SwerveDrive m_swerveDrive;
    Supplier<Pose2d> m_pose;
    double autoDuration, autoStartTime;

    public VitruvianRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, SwerveDrive swerveDrive, ArrayList<Pose2d> path, TrajectoryConfig config) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, swerveDrive);
        m_swerveDrive = swerveDrive;
        m_pose = pose;
        m_trajectory = trajectory;
        m_path = path;
        m_config = config;
    }

	@Override
    public void execute() {
        super.execute();
        SmartDashboardTab.putBoolean("DriveTrain", "isRunning", true);
    }

    @Override
    public void initialize() {
        super.initialize();
        autoStartTime = Timer.getFPGATimestamp();
        double distance = 0;
        for (int i = 0; i < m_path.size() - 1; i++) {
            var pointA = m_path.get(i);
            var pointB = m_path.get(i + 1);

            double deltaX = pointB.getTranslation().getX() - pointA.getTranslation().getX();
            double deltaY = pointB.getTranslation().getY() - pointA.getTranslation().getY();
            double deltaDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
            distance += deltaDistance;
        }
        autoDuration = (distance / m_config.getMaxVelocity()) + 2;
    }

    @Override
    public boolean isFinished() {
        double deltaX = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getX() - m_path.get(m_path.size() - 1).getTranslation().getX()));
        double deltaY = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getY() - m_path.get(m_path.size() - 1).getTranslation().getY()));
        double deltaRot = Math.abs(m_pose.get().getRotation().getDegrees() - m_path.get(m_path.size() - 1).getRotation().getDegrees());
        boolean isFinished = ((deltaX < 0.25) && (deltaY < 0.25) && (deltaRot < 4)) || (Timer.getFPGATimestamp() > (autoDuration + autoStartTime));
        SmartDashboardTab.putBoolean("DriveTrain", "Ramsete Command Finished", isFinished);
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        SmartDashboardTab.putBoolean("DriveTrain", "isRunning", false);
    }
}
