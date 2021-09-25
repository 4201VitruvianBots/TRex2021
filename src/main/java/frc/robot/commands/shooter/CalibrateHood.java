/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * An example command that uses an example subsystem.
 */
public class CalibrateHood extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Shooter m_shooter;
  private final int m_ShootingPosition;
  private final double TrenchHoodAngle = 30;
  private final double LineHoodAngle = 32;
  private final double TargetHoodAngle = 0;
  private final double TrenchRPM = 4300;
  private final double LineRPM = 4300;
  private final double TargetRPM = 4300;

  /**
   * Creates a new ExampleCommand.
   *
   */
  public CalibrateHood(Shooter shooter, int ShooterPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_ShootingPosition = ShooterPosition;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_ShootingPosition) {
      case 0:
        // Shooting from the trench
        m_shooter.setHoodAngleToSmartDashboardValue();
        m_shooter.setRPM(TrenchRPM);
        break;
      case 1:
        // Shooting from the initiation line
        m_shooter.setHoodAngleToSmartDashboardValue();
        m_shooter.setRPM(LineRPM);
        break;
      case 2:
        // Shooting from the initiation line
        m_shooter.setHoodAngleToSmartDashboardValue();
        m_shooter.setRPM(TargetRPM);
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
    m_shooter.setHoodAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
