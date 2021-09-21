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
    private final Joystick m_controller;
    double setpoint;
    private final double deadZone = 0.5;
    boolean operatorControl, usingVisionSetpoint;

    /**
     * Creates a new ExampleCommand.
     */
    public SetTurretSetpointFieldAbsolute(Turret turretSubsystem, SwerveDrive swerveDriveSubsystem, Vision visionSubsystem,
                                          JoystickWrapper controller) {
        m_turret = turretSubsystem;
        m_swerveDrive = swerveDriveSubsystem;
        m_vision = visionSubsystem;
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
        if (m_turret.getControlMode() == 1) {
            // If driver is controlling turret
            operatorControl = (Math.pow(m_controller.getRawAxis(0), 2) + Math.pow(m_controller.getRawAxis(1), 2)) >= Math.pow(deadZone, 2);

            // TODO: Add fine adjustment mode when shooting?
            if (operatorControl) {
                joystickMoved = true;
                usingVisionSetpoint = false;

                setpoint = Math.toDegrees(Math.atan2(m_controller.getRawAxis(0), -m_controller.getRawAxis(1)));
            } else if (m_vision.hasTarget() && !operatorControl) {
                // if camera has vision target, use that
                usingVisionSetpoint = true;

                setpoint = m_turret.getTurretAngle() + m_vision.getGoalX();
            } else if (joystickMoved && !operatorControl) {
                // Otherwise keep current position
                setpoint = m_turret.getTurretAngle();
                joystickMoved = false;
            }
            setpoint = Math.min(Math.max(setpoint, m_turret.getMinAngle()), m_turret.getMaxAngle());
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