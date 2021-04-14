/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


public class SetIntakeManual extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final Indexer m_indexer;

    /**
    * This function controls the intake and indexer.
    * @param intake determines the precentage at which the intake motor runs at.
    * @param indexer determines the precentage at which the indexer motor runs at.
    */
    public SetIntakeManual(Intake intake, Indexer indexer) {
        m_intake = intake;
        m_indexer = indexer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
        addRequirements(indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerOutput(1);
        m_intake.setIntakePercentOutput(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.setIndexerOutput(0);
        m_intake.setIntakePercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
