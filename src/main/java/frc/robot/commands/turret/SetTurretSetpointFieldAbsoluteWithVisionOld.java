/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetTurretSetpointFieldAbsoluteWithVisionOld extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret m_turret;
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;
  private DoubleSupplier m_xInput;
  private DoubleSupplier m_yInput;
  double setpoint, radians;
  private final double deadZone = 0.1;
  private Timer timer = new Timer();
  boolean timeout = false;
  boolean movedJoystick = false;

  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public SetTurretSetpointFieldAbsoluteWithVisionOld(Turret turretSubsystem, SwerveDrive swerveDriveSubsystem, Vision visionSybsystem, DoubleSupplier xInput, DoubleSupplier yInput) {
    m_turret = turretSubsystem;
    m_swerveDrive = swerveDriveSubsystem;
    m_vision = visionSybsystem;
    m_xInput = xInput;
    m_yInput = yInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
//    addRequirements(driveTrainSubsystem);
    addRequirements(visionSybsystem);
  }
  private boolean direction, directionTripped;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_turret.getControlMode() == 1) {
      if ((Math.pow(m_xInput.getAsDouble(), 2) + Math.pow(m_yInput.getAsDouble(), 2)) >= Math.pow(deadZone, 2)) {

        if(!directionTripped) {
          direction = m_yInput.getAsDouble() < 0;
          directionTripped = true;
        }

        if(direction) {
            if(m_xInput.getAsDouble() >= 0)
              setpoint = -Math.toDegrees(Math.atan2(-m_xInput.getAsDouble(), m_yInput.getAsDouble()));
            else
              setpoint = 360 - Math.toDegrees(Math.atan2(-m_xInput.getAsDouble(), m_yInput.getAsDouble()));

          if(setpoint > m_turret.getMaxAngle()) {
            setpoint -= 360;
            direction = false;
          }
        } else {
          if(m_xInput.getAsDouble() < 0)
            setpoint = Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));
          else
            setpoint = -360 + Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));

          if (setpoint < m_turret.getMinAngle()) {
            direction = true;
            setpoint += 360;
          }
        }
        movedJoystick = true;
      } else if (m_vision.hasTarget()){
        setpoint = m_turret.getTurretAngle() + m_vision.getGoalX();

        if(setpoint > m_turret.getMaxAngle())
          setpoint -= 360;
        else if(setpoint < m_turret.getMinAngle())
          setpoint += 360;
      }

      if(movedJoystick){
        movedJoystick = false;
      }

      m_turret.setAbsoluteSetpoint(setpoint);
    } else {
      m_turret.setPercentOutput(m_xInput.getAsDouble() * 0.2); //manual mode TODO: re-tune
    }
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
