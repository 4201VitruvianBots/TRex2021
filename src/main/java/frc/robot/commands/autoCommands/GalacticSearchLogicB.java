package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class GalacticSearchLogicB extends SequentialCommandGroup {
    public GalacticSearchLogicB(SwerveDrive swerveDrive, Vision vision, FieldSim fieldSim) {

        var command = new ConditionalCommand(new GalacticSearchBRed(swerveDrive, fieldSim), new GalacticSearchBBlue(swerveDrive, fieldSim), () -> false);
            // TODO: Vision function to determine path

        addCommands(command);
    }
}
