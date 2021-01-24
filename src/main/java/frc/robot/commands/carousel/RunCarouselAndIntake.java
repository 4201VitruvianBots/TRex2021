/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class RunCarouselAndIntake extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    public RunCarouselAndIntake(Intake intake, Carousel carousel) {
        m_intake = intake;
        m_carousel = carousel;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(carousel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setPercentOutput(0.5);
        m_carousel.setPercentOutput(0.5);
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setPercentOutput(0.0);
        m_carousel.setPercentOutput(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return startTime - Timer.getFPGATimestamp() < 2;
    }
}
