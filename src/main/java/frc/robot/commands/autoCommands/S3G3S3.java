package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.AutoControlledIntake;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class S3G3S3 extends SequentialCommandGroup {
    SwerveDrive swerveDrive;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    Turret turret;
    Vision vision;

    public S3G3S3(SwerveDrive swerveDrive, Intake intake, Indexer indexer, Shooter shooter, Turret turret, Vision vision) {
        this.swerveDrive = swerveDrive;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.vision = vision;
        
        addCommands(
            new ResetOdometry(swerveDrive),
            new AutoUseVisionCorrection(turret, vision)
        );

        shoot();

        parallel(
            new TimedIntake(intake, indexer, 3),
            new DriveBackwardDistance(swerveDrive, 3)
        );
        
        // I can't tell which command to use
        // parallel(
        //     new AutoControlledIntake(intake, indexer),
        //     new DriveBackwardDistance(swerveDrive, 3)
        // );

        addCommands(
            new DriveForwardDistance(swerveDrive, 3)
        );

        shoot();
    }

    // Routine to shoot from the Initiation line
    void shoot() {
        addCommands(
            new SetAndHoldRpmSetpoint(shooter, vision, 3500),
            new ConditionalCommand(
                new WaitCommand(0),
                new WaitCommand(0.25),
                shooter::getCanShoot
            ),
            new AutoRapidFireSetpoint(shooter, indexer, intake, 0.5).withTimeout(3),
            new SetAndHoldRpmSetpoint(shooter, vision, 0)
        );
    }
}
