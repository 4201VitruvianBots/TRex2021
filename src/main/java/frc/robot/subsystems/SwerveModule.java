/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  int mModuleNumber;

  public final TalonFX mTurningMotor;
  public final TalonFX mDriveMotor;
  double mZeroOffset;
  boolean mInverted;

  static double kF;
  static double kP;
  static double kI;
  static double kD;
  int kI_Zone = 900;
  int kMaxIAccum = 1000000;
  int kErrorBand = 50;

  int kCruiseVelocity = 14000;
  int kMotionAcceleration = kCruiseVelocity * 10;

  private double kS = 0.19;
  private double kV = 2.23;
  private double kA = 0.0289;

  private double mLastError = 0, mLastTargetAngle = 0, mTargetAngle, mVelocity = 0;

  private static final long STALL_TIMEOUT = 2000;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private double m_turnOutput;
  private double m_driveOutput;

  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private final PIDController m_drivePIDController = new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(Constants.ModuleConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.587, 2.3, 0.0917);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private Encoder simulationTurnEncoder;
  private EncoderSim simulationTurnEncoderSim;

  private SwerveModuleState swerveModuleState = new SwerveModuleState(0, new Rotation2d());
  private SwerveModuleState simulatedSwerveInput = new SwerveModuleState(0, new Rotation2d());

  private double timestamp, lastTimestamp;

  private SwerveModuleState m_desiredState = new SwerveModuleState();

  public SwerveModule(int moduleNumber, TalonFX TurningMotor, TalonFX driveMotor, double zeroOffset, boolean invertTurn, boolean invertThrottle) {
    mModuleNumber = moduleNumber;
    mTurningMotor = TurningMotor;
    mDriveMotor = driveMotor;
    mZeroOffset = zeroOffset;

    mTurningMotor.configFactoryDefault();
    mTurningMotor.configOpenloopRamp(0.1);
    mTurningMotor.configClosedloopRamp(0.1);

    mTurningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mTurningMotor.setInverted(invertTurn);
    mDriveMotor.setInverted(invertThrottle);
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
//    mTurningMotor.setSensorPhase(invertTurn);
//    mDriveMotor.setSensorPhase(!invertThrottle);

    mTurningMotor.config_kF(0,kF);
    mTurningMotor.config_kP(0,kP);
    mTurningMotor.config_kI(0,kI);
    mTurningMotor.config_IntegralZone(0, kI_Zone);
    mTurningMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
    mTurningMotor.config_kD(0,kD);
    mTurningMotor.configMotionCruiseVelocity(kCruiseVelocity);
    mTurningMotor.configMotionAcceleration(kMotionAcceleration);
    mTurningMotor.configAllowableClosedloopError(0, kErrorBand);

    mTurningMotor.setNeutralMode(NeutralMode.Brake);
    mDriveMotor.setNeutralMode(NeutralMode.Brake);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    switch(moduleNumber) {
      case 3:
        simulationTurnEncoder = new Encoder(15,14);
        break;
      case 2:
        simulationTurnEncoder = new Encoder(13,12);
        break;
      case 1:
        simulationTurnEncoder = new Encoder(11,10);
        break;
      case 0:
        simulationTurnEncoder = new Encoder(9,8);
        break;
    }
    simulationTurnEncoder.setDistancePerPulse(1);
    simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders() {
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  public Rotation2d getHeading() {
    return new Rotation2d(getTurningRadians());
  }

  public EncoderSim getEncoderSim() {
    return simulationTurnEncoderSim;
  }

  public void setTurnEncoderSimAngle(double angle) {
    simulationTurnEncoderSim.setDistance(angle);
  }

  public void setTurnEncoderSimRate(double rate) {
    simulationTurnEncoderSim.setRate(rate);
  }
  /**
   * Returns the current angle of the module.
   *
   * @return The current angle of the module in radians.
   */
  public double getTurningRadians() {
    return mTurningMotor.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningEncoderDistancePerPulse;
  }

  public double getTurnAngle() {
    return getTurningRadians() * 180.0 / Math.PI;
  }


  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  public double getVelocity() {
    return mDriveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse * 10;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), new Rotation2d(getTurningRadians()));
  }


  /**
   * The last angle setpoint of the module.
   *
   * @return The last angle setpoint of the module.
   */
  public double getTargetAngle() {
    return mLastTargetAngle;
  }
  /**
   * The last angle setpoint of the module.
   *
   * @return The last angle setpoint of the module.
   */
  public Rotation2d getTargetHeading() {
    return m_desiredState.angle;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    m_desiredState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));
    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    m_driveOutput = m_drivePIDController.calculate(
            getVelocity(), outputState.speedMetersPerSecond);

    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    m_turnOutput = m_turningPIDController.calculate(getTurningRadians(), outputState.angle.getRadians());

    double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

//    driveOutput=0;
//    System.out.println("Turn PID Output: " + turnOutput);
//    m_driveOutput = Math.signum(m_driveOutput) * Math.min(Math.abs(m_driveOutput), 0.1);
//    m_turnOutput = Math.signum(m_turnOutput) * Math.min(Math.abs(m_turnOutput), 0.4);

    System.out.println("Turn Setpoint " + mModuleNumber+ ": " +m_desiredState.angle.getDegrees());
    System.out.println("Turn Position " + mModuleNumber+ ": " +getTurnAngle());
    System.out.println("Turn Output " + mModuleNumber+ ": " +m_turnOutput);
    mDriveMotor.set(ControlMode.PercentOutput,(m_driveOutput));
    mTurningMotor.set(ControlMode.PercentOutput, m_turnOutput);

    setSimulationInput(m_driveOutput, m_turnOutput);
  }

  public void setPercentOutput(double speed) {
    mDriveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBrakeMode(boolean mode) { // True is brake, false is coast
    mDriveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    mTurningMotor.setNeutralMode(NeutralMode.Brake);
  }

  public TalonFX getTurningMotor() {
    return mTurningMotor;
  }

  public TalonFX getDriveMotor() {
    return mDriveMotor;
  }


  private void updateSmartDashboard() {
    SmartDashboardTab.putNumber("SwerveDrive","Turning PID " + mModuleNumber, turnOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
