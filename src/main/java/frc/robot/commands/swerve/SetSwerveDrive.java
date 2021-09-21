/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetSwerveDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;
  private final Intake m_intake;
  private final Vision m_vision;
  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private final boolean m_isFieldRelative;
  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDrive(SwerveDrive swerveDriveSubsystem, Intake intake, Vision vision, DoubleSupplier throttleInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, boolean isFieldRelative) {
    m_swerveDrive = swerveDriveSubsystem;
    m_intake = intake;
    m_vision = vision;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_isFieldRelative = isFieldRelative;
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
    double throttle = Math.abs(m_throttleInput.getAsDouble()) > 0.05 ? m_throttleInput.getAsDouble() : 0;
    double strafe = Math.abs(m_strafeInput.getAsDouble()) > 0.05 ? m_strafeInput.getAsDouble() : 0;
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;

    if(m_intake.getIntakingState()) {
      throttle *= -1;
      strafe *= -1;
      rotation *= -1;

      double setpoint = m_swerveDrive.getHeadingDegrees();
      if (m_vision.getIntakingPowercellCount() > 0) {
        double powercellAngle = m_vision.getIntakingPowercellAngles()[0];
        setpoint -= Math.abs(powercellAngle) > 15 ? 1 : 0;
      }
      m_swerveDrive.setAngleSetpoint(setpoint, true);
    }

    m_swerveDrive.drive(throttle, strafe, rotation,m_isFieldRelative, false);    // Forward/Back Trottle, Left/Right Strafe, Left/Right Turn
//    if(RobotBase.isReal())
//      m_swerveDrive.drive(m_leftY.getAsDouble(), m_leftX.getAsDouble(),m_rightX.getAsDouble(),false);
//    else
//      m_swerveDrive.drive(-m_leftY.getAsDouble(), m_leftX.getAsDouble(),m_rightX.getAsDouble(),false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setAngleSetpoint(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
