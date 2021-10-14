package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.commands.shooter.AutoRapidFireSetpoint;
import frc.robot.commands.shooter.SetAndHoldRpmSetpoint;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.commands.swerve.SetModuleStates;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class S3G3S3 extends SequentialCommandGroup {
    SwerveDrive m_swerveDrive;
    Intake m_intake;
    Indexer m_indexer;
    Shooter m_shooter;
    Turret m_turret;
    Vision m_vision;

    public S3G3S3(SwerveDrive swerveDrive, Intake intake, Indexer indexer, Shooter shooter, Turret turret, Vision vision) {
        m_swerveDrive = swerveDrive;
        m_intake = intake;
        m_indexer = indexer;
        m_shooter = shooter;
        m_turret = turret;
        m_vision = vision;

        SwerveModuleState[] initialStates = new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        // Initialization
        addCommands(
            new ResetOdometry(m_swerveDrive),
            new SetModuleStates(m_swerveDrive, initialStates).andThen(new WaitCommand(0.1))
        );

        // Shoot our 3 preloaded cells
        shoot();

        // Get 3 more and return
        addCommands(
            // Drive backwards while intaking
            new ParallelDeadlineGroup(
                new DriveBackwardDistance(m_swerveDrive, 5),
                new TimedIntake(m_intake, m_indexer, 3)
            )
            .andThen(() -> m_intake.setIntakePiston(false)),

            // Drive back and stop
            new DriveForwardDistance(m_swerveDrive, 5)
            .andThen(() -> m_swerveDrive.drive(0, 0, 0, false, false))
        );

        // Shoot our new power cells
        shoot();
    }

    // Routine to shoot from the Initiation line
    void shoot() {
        addCommands(
            new AutoUseVisionCorrection(m_turret, m_vision, -5),
            new SetAndHoldRpmSetpoint(m_shooter, m_vision, 3500),

            new ConditionalCommand(
                new WaitCommand(0),
                new WaitCommand(0.25),
                m_shooter::getCanShoot
            ),

            new AutoRapidFireSetpoint(m_shooter, m_indexer, m_intake, 0.5).withTimeout(3),
            new SetAndHoldRpmSetpoint(m_shooter, m_vision, 0)
        );
    }
}
