/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.indexer.SetCaroselOutput;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.intake.SetIntakePiston;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.shooter.SetUptake;
import frc.robot.commands.shooter.TestShooter;
import frc.robot.commands.shooter.FeedShooter;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.commands.turret.ZeroTurretEncoder;
import frc.robot.commands.climber.DisableClimbMode;
import frc.robot.commands.climber.EnableClimbMode;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.RetractClimber;
import frc.robot.commands.swerve.SetSwerveDriveWithAngle;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimulationReferencePose;
import frc.robot.subsystems.*;
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
  private final Turret m_turret = new Turret(m_swerveDrive);
  private final Vision m_vision = new Vision(m_swerveDrive, m_turret);
  private final Shooter m_shooter = new Shooter(m_vision, pdp);
  private final Climber m_climber = new Climber();

  private FieldSim m_FieldSim;
  private SimulationReferencePose m_referencePose;

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

  private static boolean init = false;

  public enum SkillsChallengeSelector {
    ACCURACY_CHALLENGE,
    AUTO_NAV_SLALOM,
    AUTO_NAV_BARREL,
    AUTO_NAV_BOUNCE,
    GALACTIC_SEARCH,
    GALACTIC_SEARCH_A,
    GALACTIC_SEARCH_B,
    None
}

private SkillsChallengeSelector selectedSkillsChallenge = SkillsChallengeSelector.AUTO_NAV_SLALOM; // Change this depending on the challenge

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser.setDefaultOption("Drive Straight", CommandSelector.DRIVE_STRAIGHT.ordinal());
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

  public static boolean getInitializationState() {
    return init;
  }

  public static void setInitializationState(boolean state) {
    init = state;
  }

  public void initializeSubsystems() {
    m_FieldSim = new FieldSim(m_swerveDrive, m_turret, m_shooter);
    m_referencePose = new SimulationReferencePose(m_FieldSim);

//    m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
//            () -> leftJoystick.getRawAxis(0), //left x
//            () -> leftJoystick.getRawAxis(1), //left y
//            () -> rightJoystick.getRawAxis(0))); //right x

//    if(RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
              () -> leftJoystick.getRawAxis(1), //left x
              () -> -leftJoystick.getRawAxis(0), //left y
              () -> rightJoystick.getRawAxis(0))); //right x
//    } else {
//      m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive,
//              () -> testController.getRawAxis(1), //left y
//              () -> testController.getRawAxis(0), //left x
//              () -> testController.getRawAxis(2))); //right x
//    m_swerveDrive.setDefaultCommand(new TestSwerveModule(m_swerveDrive,
//            () -> testController.getRawAxis(0),
//            () -> testController.getRawAxis(1),
//            1));
//    }
      m_turret.setDefaultCommand(new SetTurretSetpointFieldAbsolute(m_turret, m_swerveDrive, m_vision, m_shooter, xBoxController));

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
      rightJoystick.setAxisDeadband(0, 0.05);
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

      xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_shooter, 2000, true)); // A button: Flywheel low speed
      // xBoxButtons[2].whenPressed(new EnableClimbMode(m_climber, m_turret)); // X button: activate climber
      // xBoxButtons[2].whenReleased(new DisableClimbMode(m_climber, m_turret));
      xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_shooter, 3000, true)); // B button: Flywheel medium speed
      xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_shooter, 4000, true)); // Y Button: Flywheel high speed
      
      // Left xBox joystick: turret (in teleopPeriodic)

      // xBoxPOVButtons[0].whenPressed(new ExtendClimber(m_climber))//POV up: climber up
      // xBoxPOVButtons[4].whenPressed(new RetractClimber(m_climber))//POV down: climber down

      xBoxButtons[4].whenPressed(new SetIntakePiston(m_intake, true));  // Left bumper: Extend intake
      xBoxButtons[4].whenReleased(new SetIntakePiston(m_intake, false)); // Left bumper: Retract intake

      xBoxButtons[5].whileHeld(new SetCaroselOutput(m_indexer, 0.6)); // Right bumper: Spin Carousel

      xBoxLeftTrigger.whileHeld(new SetIntake(m_intake, 1)); // Left trigger: intake & carousel
      xBoxLeftTrigger.whileHeld(new SetCaroselOutput(m_indexer, 1));
      
      xBoxRightTrigger.whileHeld(new RapidFireSetpoint(m_shooter, m_indexer, m_uptake)); // Right trigger: uptake & carousel (if canShoot)
      
    }else{
      //Invert raw axis of X, Y, and rotation inputs to match WPILib convention
      testController.invertRawAxis(1, true);
      testController.invertRawAxis(0, true);
      testController.invertRawAxis(2, true);

      testController.invertRawAxis(5, true);
      for (int i = 0; i < testButtons.length; i++)
        testButtons[i] = new JoystickButton(testController, (i + 1));

//      testButtons[1].whileHeld(new FeedAll(m_indexer, m_uptake));
//      testButtons[9].whenPressed(new ToggleIntakePistons(m_intake));
//      testButtons[6].whileHeld(new ControlledIntake(m_intake, m_indexer, xBoxController)); // Deploy intake
      testButtons[1].whileHeld(new SetSwerveDriveWithAngle(m_swerveDrive,
              () -> testController.getRawAxis(1), //left y
              () -> testController.getRawAxis(0), //left x
              () -> 45)); //right x
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
//    switch (selectedSkillsChallenge) {
//      case AUTO_NAV_BARREL:
//          return new AutoNavBarrel(m_swerveDrive, m_FieldSim);
//      case AUTO_NAV_BOUNCE:
//          return new AutoNavBounce(m_swerveDrive, m_FieldSim);
//      case AUTO_NAV_SLALOM:
//          return new AutoNavSlalom(m_swerveDrive, m_FieldSim);
//      case GALACTIC_SEARCH:
//          return new SequentialCommandGroup(
//              //new SetIntakePiston(m_intake, true),
//              new GalacticSearchARed(m_swerveDrive, m_FieldSim).deadlineWith(new AutoControlledIntake(m_intake, m_indexer)),
//              new SetIntakePiston(m_intake, false));
//      case None:
//      default:
//          System.out.println("Not a recognized skills command");
//          return null;
//    return m_autoCommand;
//        return new WaitCommand(0);
//      return new TestAuto(m_swerveDrive, m_FieldSim);
//    return new AutoNavBarrel(m_swerveDrive, m_FieldSim);
      return new AutoNavSlalom(m_swerveDrive, m_FieldSim);
//    return new AutoNavBounce(m_swerveDrive, m_FieldSim);
//    return new DriveForwardDistance(m_swerveDrive, m_FieldSim, 5);
  }

  public void disabledInit() {
    setInitializationState(true);
    m_swerveDrive.setSwerveDriveNeutralMode(false); // Coast
    m_FieldSim.disabledInit();
  }

  public void robotPeriodic() {

  }

  public void teleOpInit() {
    if(RobotBase.isReal()) {
      // m_swerveDrive.resetEncoders();
      m_swerveDrive.resetOdometry(m_FieldSim.getRobotPose(), m_FieldSim.getRobotPose().getRotation());
      m_swerveDrive.setSwerveDriveNeutralMode(true); // Brake
    } else {
      m_swerveDrive.resetEncoders();
      m_swerveDrive.resetOdometry(m_FieldSim.getRobotPose(), m_FieldSim.getRobotPose().getRotation());
    }
  }

  public void teleOpPeriodic() {
    // if (xBoxController.getRawAxis(0) != 0 || xBoxController.getRawAxis(1) != 0) {
    //   m_turret.setRobotCentricSetpoint(xBoxController.getDirectionDegrees());
    // }
  }

  public void autonomousInit() {
    if (RobotBase.isReal()) {
      m_swerveDrive.resetEncoders();
      m_swerveDrive.resetOdometry(m_swerveDrive.getPose(), m_FieldSim.getRobotPose().getRotation());
      m_swerveDrive.setSwerveDriveNeutralMode(true); // Brake
    } else {
      m_FieldSim.initSim();
      m_swerveDrive.resetEncoders();
      m_swerveDrive.resetOdometry(m_FieldSim.getRobotPose(), m_FieldSim.getRobotPose().getRotation());
    }
  }

  public void autonomousPeriodic() {
  }

  public void initializeLogTopics() {
    //m_controls.initLogging();
  }

  public void simulationInit() {
    m_FieldSim.initSim();
    //m_driveTrain.setSimPose(new Pose2d(5,5, new Rotation2d()));
  }

  public void simulationPeriodic() {
    if(!RobotState.isTest())
      m_FieldSim.simulationPeriodic();
  }
}
