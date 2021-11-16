/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

public class AlignToBall extends CommandBase {
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    private final DoubleSupplier m_throttle;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;

    public AlignToBall(SwerveDrive swerveDrive, Vision vision, DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier rotation) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_swerveDrive = swerveDrive;
        this.m_vision = vision;
        this.m_throttle = throttle;
        this.m_strafe = strafe;
        this.m_rotation = rotation;
        addRequirements(this.m_swerveDrive);
        addRequirements(this.m_vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = Math.abs(m_throttle.getAsDouble()) > 0.05 ? m_throttle.getAsDouble() : 0;
        double strafe = Math.abs(m_strafe.getAsDouble()) > 0.05 ? m_strafe.getAsDouble() : 0;
        double rotation = Math.abs(m_rotation.getAsDouble()) > 0.05 ? m_rotation.getAsDouble() : 0;


        if(m_vision.hasPowerCell()) {
            double setpoint = m_swerveDrive.getHeadingDegrees() + m_vision.getPowerCellX();


            m_swerveDrive.setAngleSetpoint(setpoint, true);
        }
        else
            m_swerveDrive.setAngleSetpoint(0, false);

        m_swerveDrive.drive(throttle, strafe, rotation,true, false);

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
