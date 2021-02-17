/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Uptake;

/**
 * turn on the indexer and the uptake for a certain amount of time
 */
public class FeedAll extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Uptake m_uptake;
    double m_setpoint;
    private Timer timer = new Timer();

    private double startTime;

    public FeedAll(Indexer indexer, Uptake uptake) {
        this.m_indexer = indexer;
        this.m_uptake = uptake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerOutput(0.6);
        m_uptake.setPercentOutput(1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_indexer.setIndexerOutput(0);
        m_uptake.setPercentOutput(0);
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return timer.get() >= 2;
    }
}
