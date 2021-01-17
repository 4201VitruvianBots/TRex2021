package frc.robot.commands.autoCommands;

public class GenerateRamseteCommand {
    VitruvianRamseteCommand ramseteCommand;

//    public GenerateRamseteCommand(SwerveDrive swerveDrive, ArrayList<Pose2d> path, TrajectoryConfig config) {
//        var trajectory = TrajectoryGenerator.generateTrajectory(path, config);
//
//        ramseteCommand = new VitruvianRamseteCommand(
//                trajectory,
//                swerveDrive::getPose,
//                new RamseteController(),
//                swerveDrive.getFeedforward(),
//                swerveDrive.getSwerveDriveKinematics(),
//                swerveDrive::getSpeeds,
//                swerveDrive.getLeftPIDController(),
//                swerveDrive.getRightPIDController(),
//                swerveDrive::setVoltageOutput,
//                swerveDrive,
//                path,
//                config
//        );
//    }

    public VitruvianRamseteCommand getRamseteCommand() {
        return ramseteCommand;
    }
}