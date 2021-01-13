/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SetTurretSetpointFieldAbsoluteOld extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    private final Climber m_climber;
    private DoubleSupplier m_xInput;
    private DoubleSupplier m_yInput;
    double setpoint;
    private final double deadZone = 0.2;
    boolean timeout = false;
    boolean turning, usingVisionSetpoint;

    /**
     * Creates a new ExampleCommand.
     */
    public SetTurretSetpointFieldAbsoluteOld(Turret turretSubsystem, SwerveDrive swerveDriveSubsystem, Vision visionSubsystem,
                                             Climber climber, DoubleSupplier xInput, DoubleSupplier yInput) {
        m_turret = turretSubsystem;
        m_swerveDrive = swerveDriveSubsystem;
        m_vision = visionSubsystem;
        m_climber = climber;
        m_xInput = xInput;
        m_yInput = yInput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turretSubsystem);
    }

    private boolean direction, directionTripped, joystickMoved;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
//        SmartDashboard.putNumber("Turret X", m_xInput.getAsDouble());
//        SmartDashboard.putNumber("Turret Y", m_yInput.getAsDouble());
//        SmartDashboard.putBoolean("Joystick Moved", joystickMoved);
//        SmartDashboard.putBoolean("Vision Setpoint", usingVisionSetpoint);

        if (!m_climber.getClimbState()) {
            if (m_turret.getControlMode() == 1) {
                if ((Math.pow(m_xInput.getAsDouble(), 2) + Math.pow(m_yInput.getAsDouble(), 2)) >= Math.pow(deadZone, 2)) {
                    m_vision.ledsOn();
                    m_vision.setLastValidTargetTime();
                    joystickMoved = true;
                    if (!directionTripped) {
                        direction = m_yInput.getAsDouble() < 0;
                        directionTripped = true;
                    }

                    if (direction) {
                        if (m_xInput.getAsDouble() >= 0)
                            setpoint = -Math.toDegrees(Math.atan2(-m_xInput.getAsDouble(), m_yInput.getAsDouble()));
                        else
                            setpoint = 360 - Math.toDegrees(Math.atan2(-m_xInput.getAsDouble(), m_yInput.getAsDouble()));

                        if (setpoint > m_turret.getMaxAngle()) {
                            setpoint -= 360;
                            if (setpoint < m_turret.getMinAngle())
                                setpoint = m_turret.getMinAngle();
                            direction = false;
                        }
                    } else {
                        if (m_xInput.getAsDouble() < 0)
                            setpoint = Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));
                        else
                            setpoint = -360 + Math.toDegrees(Math.atan2(m_xInput.getAsDouble(), m_yInput.getAsDouble()));

                        if (setpoint < m_turret.getMinAngle()) {
                            direction = true;
                            setpoint += 360;
                            if (setpoint > m_turret.getMaxAngle())
                                setpoint = m_turret.getMaxAngle();
                        }
                    }
                } else if (m_vision.getValidTarget() && !joystickMoved) {
                    usingVisionSetpoint = true;
                    if (!turning) {
                        setpoint = m_turret.getTurretAngle() + m_vision.getTargetX();

                        if (setpoint > m_turret.getMaxAngle()) {
                            setpoint -= 360;
                            if (setpoint < m_turret.getMinAngle())
                                setpoint = m_turret.getMinAngle();
                            turning = true;
                        } else if (setpoint < m_turret.getMinAngle()) {
                            setpoint += 360;
                            if (setpoint > m_turret.getMaxAngle())
                                setpoint = m_turret.getMaxAngle();
                            turning = true;
                        }
                    } else {
                        if (m_turret.onTarget())
                            turning = false;
                    }
                } else if (!m_vision.getValidTarget() && !joystickMoved) {
                    usingVisionSetpoint = false;
                    setpoint = m_turret.getTurretAngle();
                } else {
                    directionTripped = false;
                    joystickMoved = false;
                }

                m_turret.setRobotCentricSetpoint(setpoint);
            } else {
                m_turret.setPercentOutput(m_xInput.getAsDouble() * 0.2); //manual mode
            }
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