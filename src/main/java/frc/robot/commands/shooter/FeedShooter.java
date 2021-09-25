/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Shooter;


/**
 * Runs the uptake and indexer if the shooter can shoot (has target and flywheel is at proper speed)
 */
public class FeedShooter extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Uptake m_uptake;
    private final Indexer m_indexer;
    private final Shooter m_shooter;

    /**
     * Creates a new FeedShooter.
     *
     * @param subsystem The subsystem used by this command.
     */

    public FeedShooter(Uptake uptake, Indexer indexer, Shooter shooter) {
        m_uptake = uptake;
        m_indexer = indexer;
        m_shooter = shooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_uptake);
        addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shooter.getCanShoot()) { // Only runs  uptake and indexer if the shooter is able to shoot
            m_uptake.setPercentOutput(1);
            m_indexer.setIndexerOutput(0.2);
        } else { // Sets output to 0 if the shooter can no longer shoot
            m_uptake.setPercentOutput(0);
            m_indexer.setIndexerOutput(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { // Sets output to 0 when command is finished
        m_uptake.setPercentOutput(0);
        m_indexer.setIndexerOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
