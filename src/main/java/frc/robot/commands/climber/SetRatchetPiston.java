/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.vitruvianlib.utils.JoystickWrapper;

/**
 * An example command that uses an example subsystem.
 */
public class SetRatchetPiston extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;
  private final boolean m_extend;
  private double timestamp;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetRatchetPiston(Climber climber, boolean extend) {
    m_climber = climber;
    m_extend = extend;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setRatchetPiston(m_extend);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if((Timer.getFPGATimestamp() - timestamp) < 0.2) {
    //   //rotate the motor counter clockwise to nick the ratchet
    //   m_climber.setClimberOutput(-0.25);
    // } else
    //   m_climber.setClimberOutput(0);
//    SmartDashboardTab.putString("Climber", "EnableClimbMode", "Executing");
//    SmartDashboardTab.putNumber("Climber", "Time Delta", (Timer.getFPGATimestamp() - timestamp));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop the motor
    // m_climber.setClimberOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;// (Timer.getFPGATimestamp() - timestamp) > 0.2;
  }
}
