/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StartShooterMotors extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private double time;
  private boolean printed = false;
  /**
   * Sets the Shooter Subsystem's power.
   *
   * @param RobotContainer.m_shooter The subsystem used by this command.
   */
  public StartShooterMotors(Shooter shooter) {
    /**
     * Use addRequirements() here to declare subsystem dependencies.
     * @param declaring the subsystem
     */
    m_shooter = shooter;
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
    /**
     * sets the power to 0 for the shooter's rpm
     */
    m_shooter.setRPM(0);
  }

  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    /**
     * sets the power to 0 for the shooter
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
