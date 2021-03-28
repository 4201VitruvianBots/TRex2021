package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;


public class EnableClimbMode extends SequentialCommandGroup {

    /**
    * allows for the climber to extend/retract.
    *@param climber allows them to be extended.
    *@param turret allows for the turret to rotate reletave to the turret command.
    */
    public EnableClimbMode(Climber climber, Turret turret) {
        addCommands(new SetTurretRobotRelativeAngle(turret,95),
                    new SetTurretControlMode(turret, 0),
                    new SetClimbMode(climber, true),
                    new ExtendClimber(climber));
    }

}
