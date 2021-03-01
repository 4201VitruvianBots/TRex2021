/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.commands.autoCommands.DriveStraight;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Uptake;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

import java.util.Map;

import static java.util.Map.entry;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems and commands are defined here...
  private final PowerDistributionPanel pdp = new PowerDistributionPanel();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final Uptake m_uptake = new Uptake();
  private final SwerveDrive m_swerveDrive = new SwerveDrive(pdp);


  private enum CommandSelector {
    DRIVE_STRAIGHT
  }

  SendableChooser<Integer> m_autoChooser = new SendableChooser();

  private final SelectCommand m_autoCommand;

  static JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
  static JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
  static JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
  static JoystickWrapper testController = new JoystickWrapper(3);
  public Button[] testButtons = new Button[10];
  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser.addDefault("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
    for (Enum commandEnum : CommandSelector.values())
      if (commandEnum != CommandSelector.DRIVE_STRAIGHT)
        m_autoChooser.addOption(commandEnum.toString(), commandEnum.ordinal());

    SmartDashboard.putData(m_autoChooser);

    m_autoCommand = new SelectCommand(
            Map.ofEntries(
//                    entry(CommandSelector.SHOOT_AND_DRIVE_BACK, new ShootAndDriveBack(m_driveTrain,m_intake,m_indexer,m_turret,m_shooter,m_vision)),
                    entry(CommandSelector.DRIVE_STRAIGHT, new DriveStraight(m_swerveDrive))
//                        entry(CommandSelector.TEST_SEQUENTIAL_REVERSE_AUTO, new TestSequentialSwitching(m_driveTrain))
            ),
            this::selectCommand
    );

    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeSubsystems() {
//    m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
//            () -> leftJoystick.getRawAxis(0), //left x
//            () -> leftJoystick.getRawAxis(1), //left y
//            () -> rightJoystick.getRawAxis(0))); //right x

    m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
            () -> testController.getRawAxis(0), //left x
            () -> testController.getRawAxis(1), //left y
            () -> testController.getRawAxis(2))); //right x
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if(RobotBase.isReal()) {
      leftJoystick.invertRawAxis(1, true);
      rightJoystick.invertRawAxis(0, true);
      xBoxController.invertRawAxis(1, true);
      xBoxController.invertRawAxis(5, true);
      for (int i = 0; i < leftButtons.length; i++)
        leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
      for (int i = 0; i < rightButtons.length; i++)
        rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
      for (int i = 0; i < xBoxButtons.length; i++)
        xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
      for (int i = 0; i < xBoxPOVButtons.length; i++)
        xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
      xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
      xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);
    }else{
      testController.invertRawAxis(1, true);
      testController.invertRawAxis(5, true);
      for (int i = 0; i < testButtons.length; i++)
        testButtons[i] = new JoystickButton(testController, (i + 1));

      testButtons[1].whileHeld(new FeedAll(m_indexer, m_uptake));
      testButtons[9].whenPressed(new ToggleIntakePistons(m_intake));
      testButtons[6].whileHeld(new ControlledIntake(m_intake, m_indexer, xBoxController)); // Deploy intake
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private CommandSelector selectCommand() {
    return CommandSelector.values()[m_autoChooser.getSelected()];
  }

  public Command getAutonomousCommand() {
//    return m_autoCommand;
        return new WaitCommand(0);
  }

  public void initializeLogTopics() {
//    m_controls.initLogging();
  }
}
