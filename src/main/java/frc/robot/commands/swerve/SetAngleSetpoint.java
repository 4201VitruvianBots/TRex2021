/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

/**
 * Sets the setpoint for the swerve drive
 */
public class SetAngleSetpoint extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveDrive m_swerveDrive;
    private final DoubleSupplier m_angleSetpoint;

    /**
     * Creates a new SetAngleSetpoint.
     *
     * @param swerveDrive The swerve drive used by this command
     * @param angleSetpoint A supplier that returns the desired angle
     */

    public SetAngleSetpoint(SwerveDrive swerveDrive, DoubleSupplier angleSetpoint) {
        m_swerveDrive = swerveDrive;
        m_angleSetpoint = angleSetpoint;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_swerveDrive.setAngleSetpoint(m_angleSetpoint.getAsDouble(), true);
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
