/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class AutoRapidFireSetpoint2 extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private double startTime, m_shootTimeout;
    private boolean timerStart;

    /**
     * Creates a new ExampleCommand.
     *
     * @param RobotContainer.m_shooter The subsystem used by this command.
     */
    public AutoRapidFireSetpoint2(Shooter shooter, Indexer indexer, Intake intake, double shootTimeout) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = shooter;
        m_indexer = indexer;
        m_intake = intake;
        m_shootTimeout = shootTimeout;
        addRequirements(shooter);
        addRequirements(indexer);
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
//        if (m_shooter.canShoot() || (Timer.getFPGATimestamp() - startTime) > m_shootTimeout) {
            m_indexer.setIndexerOutput(1);
            m_indexer.setKickerOutput(1);
            m_intake.setIntakePercentOutput(1);
//        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakePercentOutput(0);
        m_indexer.setIndexerOutput(0);
        m_indexer.setKickerOutput(0);
        m_shooter.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
