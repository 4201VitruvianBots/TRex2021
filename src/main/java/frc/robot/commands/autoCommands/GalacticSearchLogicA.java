package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import java.util.List;

public class GalacticSearchLogicA extends SequentialCommandGroup {
    public GalacticSearchLogicA(SwerveDrive swerveDrive, Vision vision) {

        var command = new ConditionalCommand(new GalacticSearchARed(swerveDrive), new GalacticSearchABlue(swerveDrive), () -> false);
            // TODO: Vision function to determine path

        addCommands(command);
    }
}
