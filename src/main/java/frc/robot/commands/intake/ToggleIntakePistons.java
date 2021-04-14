/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ToggleIntakePistons extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intake;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem intake
     */
    public ToggleIntakePistons(Intake subsystem) {
        intake = subsystem;
        /**
         * Use addRequirements() here to declare subsystem dependencies.
         * @param subsystem
         */
        addRequirements(subsystem);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        /**
         * sets the intake pistion
         * @param don't get the piston's status
         */
        intake.setintakePiston(!intake.getIntakePistonExtendStatus());
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
