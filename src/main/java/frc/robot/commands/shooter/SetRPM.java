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
import frc.robot.subsystems.Shooter;

public class SetRPM extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final double m_RPM;
  private double time;
  private boolean printed = false;
  /**
   * Sets the power and rpm for the Shooter Subsystem.
   */
  public SetRPM(Shooter shooter, double RPM) {
    /**
     * Use addRequirements() here to declare subsystem dependencies.
     * @param shooter
     */
    m_shooter = shooter;
    m_RPM = RPM;
    addRequirements(shooter);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
      time = Timer.getFPGATimestamp();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    m_shooter.setRPM(m_RPM);
    /**
     * Returns the running time on the Smart Dashboard
     */
    if(m_shooter.encoderAtSetpoint(0) && printed == false){
        SmartDashboard.putNumber("Time to Setpoint", Timer.getFPGATimestamp()-time);
        printed = true;
    }
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    /**
     * Sets the power to 0 for the shooter
     */
    m_shooter.setPower(0);
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return (false);
  }
}
