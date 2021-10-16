package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;

//import frc.vitruvianlib.utils.TrajectoryUtils;

public class IntakeDriveShoot extends SequentialCommandGroup {
    public IntakeDriveShoot(SwerveDrive swerveDrive, FieldSim fieldSim, Shooter shooter, Uptake uptake, Intake intake, Indexer indexer, Turret turret, Vision vision) {
        addCommands(
            new SetIntakePiston(intake, true),
            new AutoControlledIntake(intake, indexer).withTimeout(4),
            new SetIntakePiston(intake, false),
            new DriveForwardDistance(swerveDrive, Units.feetToMeters(20)),
            new ParallelCommandGroup(
                new AutoUseVisionCorrection(turret, vision).withTimeout(1),
                new SetRpmSetpoint(shooter, 2000, true).withTimeout(8),
                new RapidFireSetpoint(shooter, indexer, uptake).withTimeout(5)
            )
        );
    }
}
