/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * An example command that uses an example subsystem.
 */
public class FeedAll extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double m_setpoint;
  private double startTime;
  public FeedAll(Indexer indexer) {
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.setIndexerOutput(0.6);
    m_indexer.setKickerOutput(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {  
    m_indexer.setKickerOutput(0);
    m_indexer.setIndexerOutput(0);
  }

  // Returns true when the command should end.

  /*
  @Override
  public boolean isFinished() {
    double time = Timer.getFPGATimestamp();
    if(m_indexer.leftButtons[2]()){
      time = Timer.getFPGATimestamp();
    }
    if(time >= 2)
      return true;
    else
      return false;
  }
  */
}
