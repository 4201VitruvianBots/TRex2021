package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class DisableClimbMode extends SequentialCommandGroup {
    public DisableClimbMode(Climber climber, Turret turret) {
        addCommands(new SetTurretControlMode(turret, 1),
                    new SetClimbMode(climber, false),
                    new RetractClimber(climber));
    }

}
