/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

  private static Encoder turnEncoderSim; 
  private static EncoderSim turnSimulatedTurnEncoder;
  private double kS = 0.19;
  private double kV = 2.23;
  private double kA = 0.0289;

  private double mLastError = 0, mLastTargetAngle = 0, mTargetAngle, mVelocity = 0;

  private static final long STALL_TIMEOUT = 2000;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private double turnOutput;
  private double driveOutput;

  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;

  private final PIDController m_drivePIDController = new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(Constants.ModuleConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, Constants.ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
//    mTurningMotor.setSensorPhase(invertTurn);
    mDriveMotor.setSensorPhase(invertThrottle);
    mTurningMotor.setInverted(invertTurn);
    mDriveMotor.setInverted(invertThrottle);

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
      turnEncoderSim = new Encoder(15,14);
      break;
      case 2:
      turnEncoderSim = new Encoder(13,12);
      break;
      case 1:
      turnEncoderSim = new Encoder(11,10);
      break;
      case 0:
      turnEncoderSim = new Encoder(9,8);
    }
    turnEncoderSim.setDistancePerPulse(360);
    turnSimulatedTurnEncoder = new EncoderSim(turnEncoderSim);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders() {
    mTurningMotor.setSelectedSensorPosition(0);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  public Rotation2d getHeading() {
    return new Rotation2d(Units.degreesToRadians(getTurnAngle())); 
  }

  public EncoderSim getEncoderSim() {
    return turnSimulatedTurnEncoder;
  }
  /**
   * Returns the current angle of the module.
   *
   * @return The current angle of the module in radians.
   */
  public double getTurningRadians() {
    if (RobotBase.isReal()) {
      return mTurningMotor.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningEncoderDistancePerPulse;
    }
    else {
      return turnEncoderSim.getDistance();
    }
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
   * Sets the target angle of the module.
   *
   * @param targetAngle desired direction of travel for the module.
   *
   * @return The angle setpoint of the module.
   * It also inverts the driveMotor if setpoint +- 180 = targetAngle
   */
  public double setTargetAngle(double targetAngle) {
    mLastTargetAngle = targetAngle;

    targetAngle %= 360; //makes 0 to 360
    targetAngle += mZeroOffset;

    double currentAngle = getTurnAngle();
    double currentAngleMod = currentAngle % 360;
    if (currentAngleMod < 0)
      currentAngleMod += 360;

    double error = currentAngle - targetAngle;

    if(error > 90 || error < -90){
      if (error > 90)
        targetAngle += 180;
      else if (error < -90)
        targetAngle -= 180;
      mDriveMotor.setInverted(!mInverted);
    } else {
      mDriveMotor.setInverted(!mInverted);
    }

    targetAngle += currentAngle - currentAngleMod;

    double currentError = error;
    if (Math.abs(currentError - mLastError) < 7.5 &&
            Math.abs(currentAngle - targetAngle) > 5) {
      if (mStallTimeBegin == Long.MAX_VALUE) mStallTimeBegin = System.currentTimeMillis();
      if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
        throw new MotorStallException(String.format("Angle motor on swerve module '%d' has stalled.", mModuleNumber));
      }
    } else {
      mStallTimeBegin = Long.MAX_VALUE;
    }
    mLastError = currentError;

    mTargetAngle = targetAngle;
    return targetAngle;
  }


  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(getTurningRadians()));

    // Calculate the drive output from the drive PID controller.
    driveOutput = m_drivePIDController.calculate(
            getVelocity(), outputState.speedMetersPerSecond);

    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    turnOutput = m_turningPIDController.calculate(getTurningRadians(), outputState.angle.getRadians());

    double turnFeedforward =
            m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

//    driveOutput=0;
//    System.out.println("Turn PID Output: " + turnOutput);
    mDriveMotor.set(ControlMode.PercentOutput,(driveOutput));
    mTurningMotor.set(ControlMode.PercentOutput,(turnOutput));
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
    SmartDashboardTab.putNumber("SwerveDrive","Turning PID " + mModuleNumber, turnOutput); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
