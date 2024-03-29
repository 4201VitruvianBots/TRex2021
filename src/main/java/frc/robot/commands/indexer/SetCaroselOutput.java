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
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class SetCaroselOutput extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_carosel;
  private final double m_output;
  private boolean printed = false;
  private Timer timer;
  private double timeStamp;
  /**
   * Creates a new ExampleCommand.
   *
   */
  public SetCaroselOutput(Indexer carosel, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_carosel = carosel;
    m_output = output;
    timer = new Timer();
    addRequirements(carosel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_carosel.setIndexerOutput(m_output);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.start();
    timeStamp = timer.getFPGATimestamp();
    while(timer.getFPGATimestamp() > timeStamp + 0.5) { // Waits for 0.5 seconds for the shooter to slow before stopping carousel
      m_carosel.setIndexerOutput(m_output);
    }
    m_carosel.setIndexerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
