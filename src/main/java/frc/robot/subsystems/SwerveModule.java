/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.vitruvianlib.utils.CTREModuleState;

import static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule extends SubsystemBase {
  int m_moduleNumber;
  double m_zeroOffset;
  boolean m_inverted;

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_angleEncoder;
  private TalonSRX m_turnMotorSim;
  private TalonSRX m_driveMotorSim;

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ksDriveVoltSecondsPerMeter, kvDriveVoltSecondsSquaredPerMeter, kaDriveVoltSecondsSquaredPerMeter);

  private double m_turnOutput;
  private double m_driveOutput;
  private double m_lastAngle;

  private double simTurnEncoderDistance;
  private double simThrottleEncoderDistance;

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;
  private EncoderSim simulationTurnEncoderSim;
  private EncoderSim simulationThrottleEncoderSim;

  private final FlywheelSim moduleRotationSimModel = new FlywheelSim(
//          LinearSystemId.identifyVelocitySystem(kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian),
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.16, 0.0348),
          DCMotor.getFalcon500(1),
          kTurningMotorGearRatio
  );

  private final FlywheelSim moduleThrottleSimModel = new FlywheelSim(
//          LinearSystemId.identifyVelocitySystem(kvDriveVoltSecondsSquaredPerMeter * 12, kaDriveVoltSecondsSquaredPerMeter * 12),
          // Sim Values
          LinearSystemId.identifyVelocitySystem(2, 1.24),
          DCMotor.getFalcon500(1),
          kDriveMotorGearRatio
  );

  Pose2d swerveModulePose = new Pose2d();

  public SwerveModule(int moduleNumber, TalonFX turnMotor, TalonFX driveMotor, CANCoder angleEncoder, double zeroOffset, boolean invertTurn, boolean invertThrottle) {
    m_moduleNumber = moduleNumber;
    m_driveMotor = driveMotor;
    m_turnMotor = turnMotor;
    m_angleEncoder = angleEncoder;
    m_zeroOffset = zeroOffset;

    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(AngleEncoderConfig);
    m_angleEncoder.configMagnetOffset(m_zeroOffset);
    m_angleEncoder.configSensorDirection(true);

    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(DriveMotorConfig);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_turnMotor.configFactoryDefault();
    m_turnMotor.configAllSettings(TurnMotorConfig);
    m_turnMotor.configRemoteFeedbackFilter(m_angleEncoder, 0, 20);
    m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    m_turnMotor.setSelectedSensorPosition((getHeadingDegrees() - zeroOffset) / kTurningEncoderDistancePerPulse);
    m_turnMotor.configPeakOutputForward(0.3);
    m_turnMotor.configPeakOutputReverse(-0.3);
    m_turnMotor.setInverted(true);
//    m_turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


    if(RobotBase.isSimulation()) {
      switch (moduleNumber) {
        case 3:
          simulationTurnEncoder = new Encoder(15, 14);
          simulationThrottleEncoder = new Encoder(0, 1);
          break;
        case 2:
          simulationTurnEncoder = new Encoder(13, 12);
          simulationThrottleEncoder = new Encoder(2, 3);
          break;
        case 1:
          simulationTurnEncoder = new Encoder(11, 10);
          simulationThrottleEncoder = new Encoder(4, 5);
          break;
        case 0:
          simulationTurnEncoder = new Encoder(9, 8);
          simulationThrottleEncoder = new Encoder(6, 7);
          break;
      }

      simulationTurnEncoder.setDistancePerPulse(kTurningEncoderDistancePerPulse);
      simulationThrottleEncoder.setDistancePerPulse(kDriveSimEncoderDistancePerPulse);

      simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
      simulationThrottleEncoderSim = new EncoderSim(simulationThrottleEncoder);

      m_driveMotorSim = new TalonSRX(driveMotor.getDeviceID());
      m_turnMotorSim = new TalonSRX(turnMotor.getDeviceID());

      m_driveMotorSim.configFactoryDefault();
      m_driveMotorSim.configAllSettings(DriveSimMotorConfig);
      m_driveMotorSim.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

      m_turnMotorSim.configFactoryDefault();
      m_turnMotorSim.configAllSettings(TurnSimMotorConfig);
      m_turnMotorSim.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }
  }

  /**
   * Zeros all the SwerveModule encoders.
   */
  public void resetEncoders() {
//    m_turnMotor.setSelectedSensorPosition(0);
    m_driveMotor.setSelectedSensorPosition(0);
    if (RobotBase.isSimulation()) {
      m_turnMotorSim.setSelectedSensorPosition(0);
      m_driveMotorSim.setSelectedSensorPosition(0);
    }
  }

  public Rotation2d getHeading() {
    return new Rotation2d(getHeadingRadians());
  }

  public double getHeadingDegrees() {
    if(RobotBase.isReal())
//      return m_turnMotor.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningEncoderDistancePerPulse;
      return m_angleEncoder.getAbsolutePosition();
    else
      try {
//        return m_turnMotorSim.getSelectedSensorPosition() * Constants.ModuleConstants.kTurningSimEncoderDistancePerPulse;
        return Units.radiansToDegrees(simulationTurnEncoder.getDistance());
      } catch (Exception e) {
        return 0;
      }
  }

  /**
   * Returns the current angle of the module.
   *
   * @return The current angle of the module in radians.
   */
  public double getHeadingRadians() {
    return Units.degreesToRadians(getHeadingDegrees());
  }

  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  public double getVelocity() {
    if(RobotBase.isReal())
      return m_driveMotor.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveEncoderDistancePerPulse * 10;
    else
//      return m_driveMotorSim.getSelectedSensorVelocity() * Constants.ModuleConstants.kDriveSimEncoderDistancePerPulse * 10;
      return simulationThrottleEncoder.getRate();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
      return new SwerveModuleState(getVelocity(), getHeading());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    // SwerveModuleState outputState = CTREModuleState.optimize(state, getState().angle);
    SwerveModuleState outputState = state;
    double angle = 0.0;

    if(RobotBase.isReal()) {
      if (isOpenLoop) {
        double percentOutput = outputState.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
      } else {
        double velocityOutput = outputState.speedMetersPerSecond / (kDriveEncoderDistancePerPulse * 10.0);
        m_driveMotor.set(ControlMode.Velocity, velocityOutput, DemandType.ArbitraryFeedForward, m_driveFeedforward.calculate(outputState.speedMetersPerSecond));
      }

      //Prevent rotating module if speed is less then 1%. Prevents Jittering.
      angle = (Math.abs(outputState.speedMetersPerSecond) <= (Constants.DriveConstants.kMaxSpeedMetersPerSecond * 0.01)) ?
              m_lastAngle : outputState.angle.getDegrees();
      m_turnMotor.set(ControlMode.Position, angle / kTurningEncoderDistancePerPulse);

    } else {
      if(isOpenLoop) {
        double percentOutput = outputState.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        m_driveMotorSim.set(ControlMode.PercentOutput, percentOutput);
      } else {
        double velocityOutput = outputState.speedMetersPerSecond / (kDriveSimEncoderDistancePerPulse * 10.0);
        m_driveMotorSim.set(ControlMode.Velocity, velocityOutput, DemandType.ArbitraryFeedForward, m_driveFeedforward.calculate(outputState.speedMetersPerSecond));
      }

      //Prevent rotating module if speed is less then 1%. Prevents Jittering.
      angle = (Math.abs(outputState.speedMetersPerSecond) <= (Constants.DriveConstants.kMaxSpeedMetersPerSecond * 0.01)) ?
              m_lastAngle : outputState.angle.getDegrees();

      m_turnMotorSim.set(ControlMode.Position, angle / kTurningSimEncoderDistancePerPulse);

      m_driveOutput = m_driveMotorSim.getMotorOutputPercent();
      m_turnOutput = m_turnMotorSim.getMotorOutputPercent();
    }
    m_lastAngle = angle;
  }

  public void setBrakeMode(boolean mode) { // True is brake, false is coast
    m_driveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    m_turnMotor.setNeutralMode(NeutralMode.Brake);
  }
  public Pose2d getPose() {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }


  private void updateSmartDashboard() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    moduleRotationSimModel.setInputVoltage(m_turnOutput * RobotController.getBatteryVoltage());
    moduleThrottleSimModel.setInputVoltage(m_driveOutput * RobotController.getBatteryVoltage());

    moduleRotationSimModel.update(0.02);
    moduleThrottleSimModel.update(0.02);

//    simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
//    simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;

//    Unmanaged.feedEnable(20);
//    m_turnMotor.getSimCollection().setQuadratureRawPosition((int) (simTurnEncoderDistance / kTurningEncoderDistancePerPulse));
//    m_turnMotor.getSimCollection().setQuadratureVelocity((int) (moduleRotationSimModel.getAngularVelocityRadPerSec() / kTurningEncoderDistancePerPulse * 10));
//    m_driveMotor.getSimCollection().setQuadratureRawPosition((int) (simThrottleEncoderDistance / kDriveEncoderDistancePerPulse));
//    m_driveMotor.getSimCollection().setQuadratureVelocity((int) (moduleThrottleSimModel.getAngularVelocityRadPerSec() / kDriveEncoderDistancePerPulse * 10));
//    m_turnMotorSim.getSimCollection().setQuadratureRawPosition((int) (simTurnEncoderDistance / kTurningEncoderDistancePerPulse));
//    m_turnMotorSim.getSimCollection().setQuadratureVelocity((int) (moduleRotationSimModel.getAngularVelocityRadPerSec() / kTurningEncoderDistancePerPulse * 10));
//    m_driveMotorSim.getSimCollection().setQuadratureRawPosition((int) (simThrottleEncoderDistance / kDriveSimEncoderDistancePerPulse));
//    m_driveMotorSim.getSimCollection().setQuadratureVelocity((int) (moduleThrottleSimModel.getAngularVelocityRadPerSec() / kDriveSimEncoderDistancePerPulse / 10));

    simTurnEncoderDistance += moduleRotationSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationTurnEncoderSim.setDistance(simTurnEncoderDistance);
    simulationTurnEncoderSim.setRate(moduleRotationSimModel.getAngularVelocityRadPerSec());

    simThrottleEncoderDistance += moduleThrottleSimModel.getAngularVelocityRadPerSec() * 0.02;
    simulationThrottleEncoderSim.setDistance(simThrottleEncoderDistance);
    simulationThrottleEncoderSim.setRate(moduleThrottleSimModel.getAngularVelocityRadPerSec());

//    System.out.println("Module " + mModuleNumber + " State: " + getState());
  }
}
