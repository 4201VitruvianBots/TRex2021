package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class GalacticSearchLogicB extends SequentialCommandGroup {
    public GalacticSearchLogicB(SwerveDrive swerveDrive, Vision vision) {

        var command = new ConditionalCommand(new GalacticSearchBRed(swerveDrive), new GalacticSearchBBlue(swerveDrive), () -> false);
            // TODO: Vision function to determine path

        addCommands(command);
    }
}
