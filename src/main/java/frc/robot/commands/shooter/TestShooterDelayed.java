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
import frc.robot.subsystems.Uptake;

/**
 * An example command that uses an example subsystem.
 */
public class TestShooterDelayed extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Intake m_intake;
  private double time;
  private boolean test, stopTest;
  private boolean printed = false;
  private final Uptake m_uptake;
  
  public TestShooterDelayed(Shooter shooter, Indexer indexer, Intake intake, Uptake uptake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_indexer = indexer;
    m_intake = intake;
    m_uptake = uptake;
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTestRPM();

    if(m_shooter.getTestRPM() != 0)
      if (Math.abs(m_shooter.getRPM(0) - m_shooter.getTestRPM()) < m_shooter.getRPMTolerance()) {
        m_indexer.setIndexerOutput(0.95);
        m_uptake.setPercentOutput(0.95);
        m_intake.setIntakePercentOutput(0.95);
//    } else if (!m_indexer.getIndexerTopSensor()) {
//      m_indexer.setIndexerOutput(1);
//      m_uptake.setPercentOutput(-0.25);
      } else {
        m_indexer.setIndexerOutput(0);
        m_uptake.setPercentOutput(0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_uptake.setPercentOutput(0);
    m_shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
