/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class TestShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Intake m_intake;
  private double timestamp;
  private boolean timerStart;
  private boolean printed = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param RobotContainer.m_shooter The subsystem used by this command.
   */
  public TestShooter(Shooter shooter, Indexer indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_indexer = indexer;
    m_intake = intake;
    addRequirements(shooter);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setTestRPM();

    if(m_shooter.getTestRPM() != 0) {


      if (Math.abs(m_shooter.getRPM(0) - m_shooter.getTestRPM()) < m_shooter.getRPMTolerance() && !timerStart) {
        timerStart = true;
        timestamp = Timer.getFPGATimestamp();
      } else if (Math.abs(m_shooter.getRPM(0) - m_shooter.getTestRPM()) > m_shooter.getRPMTolerance() && timerStart) {
        timestamp = 0;
        timerStart = false;
      }

      if (timestamp != 0)
        if (timerStart && Timer.getFPGATimestamp() - timestamp > 0.1) {
          m_indexer.setIndexerOutput(1);
          m_indexer.setKickerOutput(1);
          m_intake.setIntakePercentOutput(1);
        } else {
          m_indexer.setIndexerOutput(0);
          m_indexer.setKickerOutput(0);
          m_intake.setIntakePercentOutput(0);

        }
      //      if(!test) {
      //        test = true;
      //        time = Timer.getFPGATimestamp();
      //      } else if(!stopTest){
      //        SmartDashboard.putNumber("Recovery Time", Timer.getFPGATimestamp() - time);
      //        stopTest = true;
      //      }
      //    } else if (!m_indexer.getIndexerTopSensor()) {
      //      m_indexer.setIndexerOutput(1);
      //      m_indexer.setKickerOutput(-0.25);
      //    } else {
      //      m_indexer.setIndexerOutput(0);
      //      m_indexer.setKickerOutput(0);
    }
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
    return (false);
  }
}
