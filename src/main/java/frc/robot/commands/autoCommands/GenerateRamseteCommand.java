package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public class GenerateRamseteCommand {
    VitruvianRamseteCommand ramseteCommand;

    public GenerateRamseteCommand(SwerveDrive swerveDrive, ArrayList<Pose2d> path, TrajectoryConfig config) {
        var trajectory = TrajectoryGenerator.generateTrajectory(path, config);

        ramseteCommand = new VitruvianRamseteCommand(
                trajectory,
                swerveDrive::getPose,
                new RamseteController(),
                swerveDrive.getFeedforward(),
                swerveDrive.getSwerveDriveKinematics(),
                swerveDrive::getSpeeds,
                swerveDrive.getLeftPIDController(),
                swerveDrive.getRightPIDController(),
                swerveDrive::setVoltageOutput,
                swerveDrive,
                path,
                config
        );
    }

    public VitruvianRamseteCommand getRamseteCommand() {
        return ramseteCommand;
    }
}