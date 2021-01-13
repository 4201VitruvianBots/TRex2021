/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IncrementIndexer extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double m_setpoint;
  private double startTime;
  public IncrementIndexer(Indexer subsystem) {
    m_indexer = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    m_setpoint = m_indexer.getPosition() + 7 / (1.25 * Math.PI) * 20;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//    m_indexer.incrementIndexer(m_setpoint);
    m_indexer.setKickerOutput(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_indexer.setKickerOutput(0);
    SmartDashboard.putNumber("Execution Time", Timer.getFPGATimestamp() - startTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return m_indexer.onTarget();
    return false;
  }
}
