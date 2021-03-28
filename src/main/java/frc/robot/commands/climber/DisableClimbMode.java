package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class DisableClimbMode extends SequentialCommandGroup {

    /**
    * does not allow for the climber to extend/retract.
    *@param climber retracts the climbing pistons and doesn't allow them to be extended.
    *@param turret allows for the turret to rotate reletave to the turret command.
    */
    public DisableClimbMode(Climber climber, Turret turret) {
        addCommands(new SetTurretControlMode(turret, 1),
                    new SetClimbMode(climber, false),
                    new RetractClimber(climber));
    }

}
