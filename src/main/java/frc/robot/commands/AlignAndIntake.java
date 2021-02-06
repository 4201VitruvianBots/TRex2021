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
public class AlignAndIntake extends ParallelCommandGroup {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final Vision m_vision;
    private final SwerveDrive m_swerveDrive;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    public AlignAndIntake(Intake intake, Vision vision, SwerveDrive swerveDrive) {
        addCommands(
            new AlignToPowerCell(vision, swerveDrive),
            new SetIntake(intake, 0.5)
        );

    }
}
