/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetSwerveDriveFieldRelative extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_leftX, m_leftY, m_rightX;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveFieldRelative(SwerveDrive swerveDriveSubsystem, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rightX) {
    m_swerveDrive = swerveDriveSubsystem;
    m_leftX = xInput;
    m_leftY = yInput;
    m_rightX = rightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = Math.abs(m_leftY.getAsDouble()) > 0.05 ? m_leftY.getAsDouble() : 0;
    double strafe = Math.abs(m_leftX.getAsDouble()) > 0.05 ? m_leftX.getAsDouble() : 0;
    double rotation = Math.abs(m_rightX.getAsDouble()) > 0.05 ? m_rightX.getAsDouble() : 0;

    m_swerveDrive.drive(throttle, strafe, rotation,false);    // Forward/Back Throttle, Left/Right Strafe, Left/Right Turn
//    if(RobotBase.isReal())
//      m_swerveDrive.drive(m_leftY.getAsDouble(), m_leftX.getAsDouble(),m_rightX.getAsDouble(),false);
//    else
//      m_swerveDrive.drive(-m_leftY.getAsDouble(), m_leftX.getAsDouble(),m_rightX.getAsDouble(),false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
