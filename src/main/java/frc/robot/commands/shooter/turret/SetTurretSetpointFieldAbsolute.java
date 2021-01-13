/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/**
 * An example command that uses an example subsystem.
 */
public class SetTurretSetpointFieldAbsolute extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    private final Shooter m_shooter;
    private final Climber m_climber;
    private Joystick m_controller;
    double setpoint;
    private final double deadZone = 0.5;
    boolean timeout = false;
    boolean turning, usingVisionSetpoint;

    /**
     * Creates a new ExampleCommand.
     */
    public SetTurretSetpointFieldAbsolute(Turret turretSubsystem, SwerveDrive swerveDriveSubsystem, Vision visionSubsystem,
                                          Shooter shooter, Climber climber, Joystick controller) {
        m_turret = turretSubsystem;
        m_swerveDrive = swerveDriveSubsystem;
        m_vision = visionSubsystem;
        m_shooter = shooter;
        m_climber = climber;
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

        if (!m_climber.getClimbState()) {
            if (m_turret.getControlMode() == 1) {
                // TODO: Add fine adjustment mode when shooting?
                if ((Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2)) >= Math.pow(deadZone, 2)) {
                    m_vision.ledsOn();
                    m_vision.setLastValidTargetTime();
                    joystickMoved = true;

                    if (m_controller.getRawAxis(0) >= 0)
                        setpoint = -Math.toDegrees(Math.atan2(-m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
                    else
                        setpoint = Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), m_controller.getRawAxis(1)));

                    if(setpoint > m_turret.getMaxAngle())
                        setpoint = m_turret.getMaxAngle();

                    if(setpoint < m_turret.getMinAngle())
                        setpoint = m_turret.getMinAngle();
//                    if (!directionTripped) {
//                        direction = m_controller.getRawAxis(1) < 0;
//                        directionTripped = true;
//                    }
//
//                    if (direction) {
//                        if (m_controller.getRawAxis(0) >= 0)
//                            setpoint = -Math.toDegrees(Math.atan2(-m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
//                        else
//                            setpoint = 360 - Math.toDegrees(Math.atan2(-m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
//
//                        if (setpoint > m_turret.getMaxAngle()) {
//                            setpoint = m_turret.getMaxAngle();
////                            setpoint -= 360;
////                            if (setpoint < m_turret.getMinAngle())
////                                setpoint = m_turret.getMinAngle();
////                            direction = false;
//                        }
//                    } else {
//                        if (m_controller.getRawAxis(0) < 0)
//                            setpoint = Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
//                        else
//                            setpoint = -360 + Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), m_controller.getRawAxis(1)));
//
//                        if (setpoint < m_turret.getMinAngle()) {
//                            setpoint = m_turret.getMinAngle();
////                            setpoint += 360;
////                            if (setpoint > m_turret.getMaxAngle())
////                                setpoint = m_turret.getMaxAngle();
////                            direction = true;
//                        }
//                    }
                    if (m_vision.hasTarget() && Math.abs(m_vision.getFilteredTargetX()) < 20) {
                        m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                        m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                    }
                } else if (m_vision.hasTarget() && !joystickMoved) {
                    usingVisionSetpoint = true;
                    if (!turning) {
                        m_vision.ledsOn();
                        setpoint = m_turret.getTurretAngle() + m_vision.getTargetX();

                        if (setpoint > m_turret.getMaxAngle()) {
                            setpoint = m_turret.getMaxAngle();
//                            setpoint -= 360;
//                            if (setpoint < m_turret.getMinAngle())
//                                setpoint = m_turret.getMinAngle();
//                            turning = true;
                        } else if (setpoint < m_turret.getMinAngle()) {
                            setpoint = m_turret.getMinAngle();
//                            setpoint += 360;
//                            if (setpoint > m_turret.getMaxAngle())
//                                setpoint = m_turret.getMaxAngle();
//                            turning = true;
                        }
                    } else {
                        m_vision.ledsOff();
                        if (m_turret.onTarget())
                            turning = false;
                    }
                } else if (!m_vision.hasTarget() && !joystickMoved) {
                    usingVisionSetpoint = false;
                    setpoint = m_turret.getTurretAngle();
                } else {
                    directionTripped = false;
                    joystickMoved = false;
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                }
                
                if(m_shooter.canShoot()) {
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.8);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.8);
                } else {
                    m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                    m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                }
//                SmartDashboardTab.putNumber("Turret", "Angle Setpoint", setpoint);
                m_turret.setRobotCentricSetpoint(setpoint);
//                m_turret.setFieldCentricSetpoint(setpoint);
            } else {
                m_turret.setPercentOutput(m_controller.getRawAxis(0) * 0.2); //manual mode
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