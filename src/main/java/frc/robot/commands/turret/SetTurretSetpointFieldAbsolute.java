/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.vitruvianlib.utils.JoystickWrapper;

/**
 * An example command that uses an example subsystem.
 */
public class SetTurretSetpointFieldAbsolute extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    private final Shooter m_shooter;
    private final Joystick m_controller;
    double setpoint;
    private final double deadZone = 0.5;
    boolean timeout = false;
    boolean turning, usingVisionSetpoint;

    /**
     * Creates a new ExampleCommand.
     */
    public SetTurretSetpointFieldAbsolute(Turret turretSubsystem, SwerveDrive swerveDriveSubsystem, Vision visionSubsystem,
                                          Shooter shooter, JoystickWrapper controller) {
        m_turret = turretSubsystem;
        m_swerveDrive = swerveDriveSubsystem;
        m_vision = visionSubsystem;
        m_shooter = shooter;
        m_controller = controller;
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

        if (m_turret.getControlMode() == 1) {
            // If driver is controlling turret

            // TODO: Add fine adjustment mode when shooting?
            if ((Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2)) >= Math.pow(deadZone, 2)) {
                m_vision.setLastValidTargetTime();
                joystickMoved = true;

                setpoint = Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), -m_controller.getRawAxis(1)));

                setpoint = Math.min(Math.max(setpoint, m_turret.getMinAngle()), m_turret.getMaxAngle());

                if (m_vision.hasTarget() && Math.abs(m_vision.getFilteredTargetX()) < 20) {
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                }
            } else if (m_vision.hasTarget() && !joystickMoved) {
                // if camera has vision target
                usingVisionSetpoint = true;
                if (!turning) {
                    setpoint = m_turret.getTurretAngle() + m_vision.getGoalX();

                    if (setpoint > m_turret.getMaxAngle()) {
                        setpoint = m_turret.getMaxAngle();
                    } else if (setpoint < m_turret.getMinAngle()) {
                        setpoint = m_turret.getMinAngle();
                    }
                } else {
                    if (m_turret.onTarget())
                        turning = false;
                }
            } else if (!m_vision.hasTarget() && !joystickMoved) {
                // Otherwise don't move
                usingVisionSetpoint = false;
                setpoint = m_turret.getTurretAngle();
            } else {
                directionTripped = false;
                joystickMoved = false;
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
            }

            if (m_shooter.getCanShoot() && m_vision.hasTarget() && Math.abs(m_vision.getGoalX()) < 1) {
                // Rumble if there is a valid target and you are in the target
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.8);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.8);
            } else {
                m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
            }
//                SmartDashboardTab.putNumber("Turret", "Angle Setpoint", setpoint);
            m_turret.setRobotCentricSetpoint(setpoint);
            System.out.println("Turret Setpoint: " + setpoint);
//                m_turret.setFieldCentricSetpoint(setpoint);
        } else {
            m_turret.setPercentOutput(m_controller.getRawAxis(0) * 0.2); //manual mode
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