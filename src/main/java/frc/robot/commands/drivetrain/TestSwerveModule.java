/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class TestSwerveModule extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final DoubleSupplier m_xInput, m_yInput;
  private final int m_moduleIdx;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public TestSwerveModule(SwerveDrive swerveDriveSubsystem, DoubleSupplier xInput, DoubleSupplier yInput, int moduleIdx) {
    m_swerveDrive = swerveDriveSubsystem;
    m_xInput = xInput;
    m_yInput = yInput;
    m_moduleIdx = moduleIdx;
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
    double yInput = Math.abs(m_yInput.getAsDouble()) > 0.05 ? -m_yInput.getAsDouble() : 0;
    double xInput = Math.abs(m_xInput.getAsDouble()) > 0.05 ? m_xInput.getAsDouble() : 0;

    var inputState = new SwerveModuleState(0, new Rotation2d(Math.atan2(yInput, xInput)));
    m_swerveDrive.getSwerveModule(m_moduleIdx).setDesiredState(inputState);

    System.out.println("Input State: " + inputState);
    System.out.println("Output State: " + m_swerveDrive.getSwerveModule(m_moduleIdx).getState());
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
