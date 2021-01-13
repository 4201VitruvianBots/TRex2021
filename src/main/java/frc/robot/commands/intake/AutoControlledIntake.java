/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import com.team254.lib.util.MinTimeBoolean;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.swing.*;

/**
 * An example command that uses an example subsystem.
 */
public class AutoControlledIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Intake m_intake;

  private double intakeRPM = 5000;
  private double indexRPM = 300;
  private double timestamp, intakeTimestamp, indexerTimestamp, fourBallTimestamp;
  private boolean intaking, haveFour, haveFourTripped;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoControlledIntake(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakingState(true);
    timestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_intake.setIntakePercentOutput(0);
        m_indexer.setKickerOutput(0);
        if (!intaking) {
          //indexerTimestamp = Timer.getFPGATimestamp();
          intaking = true;
          m_indexer.setIndexerOutput(0.95);
//          m_indexer.setRPM(indexRPM);
        } else {
          intaking = false;
          m_indexer.setIndexerOutput(0);
//          m_indexer.setRPM(0);
        }

//        if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor() && !haveFourTripped) {
//          fourBallTimestamp = Timer.getFPGATimestamp();
//          haveFourTripped = true;
//        } else if(!m_indexer.getIndexerBottomSensor() || !m_indexer.getIndexerTopSensor()){
//          fourBallTimestamp = 0;
//          haveFourTripped = false;
//          haveFour = false;
//        }
    }

    //updateTimedRollers();
  

  private void updateTimedRollers() {
    timestamp = Timer.getFPGATimestamp();
      if(indexerTimestamp != 0)
        if(timestamp - indexerTimestamp < 0.1)
          m_indexer.setRPM(indexRPM);
        else {
          m_indexer.setRPM(0);
          intaking = false;
        }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakingState(false);
    m_intake.setIntakePercentOutput(0);
    m_indexer.setIndexerOutput(0);
    m_indexer.setKickerOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
