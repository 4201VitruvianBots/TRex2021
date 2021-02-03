/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

/**
 * An example command that uses an example subsystem.
 */
public class AlignToPowerCell extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Vision m_vision;
    private final SwerveDrive m_swerveDrive;
    private final double PowerCellX;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    public AlignToPowercell(Vision vision, SwerveDrive swerveDrive) {
        m_vision = vision;
        m_swerveDrive = swerveDrive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        powerCellX = Vision.getPowerCellX();
        if (powerCellX > 0) {
            SetSwerveDrive(m_swerveDrive, -0.5, 0, 0);
        } else {
            SetSwerveDrive(m_swerveDrive, 0.5, 0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SetSwerveDrive(m_swerveDrive, 0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return powerCellX - 5 < 0;
        // 5 is placeholder value, equal to pixels offset of camera from center
    }
}
