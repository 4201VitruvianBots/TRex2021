package frc.robot.simulation;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N7;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;

public class SimulateSwerveDrive {
  private final DCMotor m_motor;
  private final double m_originalGearing;
  private final Matrix<N7, N1> m_measurementStdDevs;
  private double m_currentGearing;
  private final double m_wheelRadiusMeters;

  private final LinearSystem<N2, N2, N2> m_plant;

  private final double robotWidth, robotLength;

  private DifferentialDriveKinematics xKinematics, yKinematics, rotKinematics;
  private DifferentialDrivetrainSim xSim, ySim, rotationSim;
  private SwerveModuleTurnSim[] simulatedSwerveModules = new SwerveModuleTurnSim[4];


  private double xDistance, yDistance, rotationDistance;
  private double xVel, yVel, rotationVel, angularVel;
  private Pose2d swervePose;

  private Encoder xEncoder, yEncoder, rotationEncoder;
  private EncoderSim xSimEncoder, ySimEncoder, rotationSimEncoder;
  private ADXRS450_Gyro gyro;
  private ADXRS450_GyroSim gyroSim;

  private SwerveModuleState[] m_inputSwerveStates = new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
  };

  private SwerveModuleState[] targetStates = new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
  };

  private ChassisSpeeds m_lastChassisSpeed = new ChassisSpeeds();
  private double lastRotationSpeed;
  private double m_lastTimestamp;
  private double rotSetpoint,m_lastRotSetpoint, m_lastInputAngle;
  private double xInputVoltage, yInputVoltage, rotInputVoltage;
  private Rotation2d rotationHeading = new Rotation2d();

  private boolean isFieldRelative;

  private SwerveModule[] m_swerveModules = new SwerveModule[4];
  /**
   * Create a SimDrivetrain.
   *
   * @param driveMotor A {@link DCMotor} representing the left side of the drivetrain.
   * @param gearing The gearing ratio between motor and wheel, as output over input. This must be
   *     the same ratio as the ratio used to identify or create the drivetrainPlant.
   * @param jKgMetersSquared The moment of inertia of the drivetrain about its center.
   * @param massKg The mass of the drivebase.
   * @param wheelRadiusMeters The radius of the wheels on the drivetrain.
   * @param trackWidthMeters The robot's track width, or distance between left and right wheels.
   * @param measurementStdDevs Standard deviations for measurements, in the form [x, y, heading,
   *     left velocity, right velocity, left distance, right distance]^T. Can be null if no noise is
   *     desired. Gyro standard deviations of 0.0001 radians, velocity standard deviations of 0.05
   *     m/s, and position measurement standard deviations of 0.005 meters are a reasonable starting
   *     point.
   */
  @SuppressWarnings("ParameterName")
  public SimulateSwerveDrive(
      DCMotor driveMotor,
      double gearing,
      double jKgMetersSquared,
      double massKg,
      double wheelRadiusMeters,
      double trackWidthMeters,
      double robotLengthMeters,
      Matrix<N7, N1> measurementStdDevs) {
    this(
        LinearSystemId.createDrivetrainVelocitySystem(
            driveMotor,
            massKg,
            wheelRadiusMeters,
            trackWidthMeters,
            jKgMetersSquared,
            gearing),
        driveMotor,
        gearing,
        trackWidthMeters,
        robotLengthMeters,
        wheelRadiusMeters,
        measurementStdDevs);
  }

  /**
   * Create a SimDrivetrain .
   *
   * @param drivetrainPlant The {@link LinearSystem} representing the robot's drivetrain. This
   *     system can be created with {@link
   *     edu.wpi.first.wpilibj.system.plant.LinearSystemId#createDrivetrainVelocitySystem(DCMotor,
   *     double, double, double, double, double)} or {@link
   *     edu.wpi.first.wpilibj.system.plant.LinearSystemId#identifyDrivetrainSystem(double, double,
   *     double, double)}.
   * @param driveMotor A {@link DCMotor} representing the drivetrain.
   * @param gearing The gearingRatio ratio of the robot, as output over input. This must be the same
   *     ratio as the ratio used to identify or create the drivetrainPlant.
   * @param trackWidthMeters The distance between the two sides of the drivetrian. Can be found with
   *     frc-characterization.
   * @param wheelRadiusMeters The radius of the wheels on the drivetrain, in meters.
   * @param measurementStdDevs Standard deviations for measurements, in the form [x, y, heading,
   *     left velocity, right velocity, left distance, right distance]^T. Can be null if no noise is
   *     desired. Gyro standard deviations of 0.0001 radians, velocity standard deviations of 0.05
   *     m/s, and position measurement standard deviations of 0.005 meters are a reasonable starting
   *     point.
   */
  public SimulateSwerveDrive(
      LinearSystem<N2, N2, N2> drivetrainPlant,
      DCMotor driveMotor,
      double gearing,
      double trackWidthMeters,
      double robotLengthMeters,
      double wheelRadiusMeters,
      Matrix<N7, N1> measurementStdDevs) {
    m_plant = drivetrainPlant;
    robotWidth = trackWidthMeters;
    robotLength = robotLengthMeters;
    m_motor = driveMotor;
    m_originalGearing = gearing;
    m_measurementStdDevs = measurementStdDevs;
    m_wheelRadiusMeters = wheelRadiusMeters;
    m_currentGearing = m_originalGearing;

    xSim = new DifferentialDrivetrainSim(
        drivetrainPlant,
        driveMotor,
        gearing,
        robotWidth,
        wheelRadiusMeters,
        measurementStdDevs
    );
    xKinematics = new DifferentialDriveKinematics(robotWidth);
    ySim = new DifferentialDrivetrainSim(
        drivetrainPlant,
        driveMotor,
        gearing,
        robotLength,
        wheelRadiusMeters,
        measurementStdDevs
    );
    ySim.setPose(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(90))));
    yKinematics = new DifferentialDriveKinematics(robotLength);
    rotationSim = new DifferentialDrivetrainSim(
        drivetrainPlant,
        driveMotor,
        gearing,
        robotWidth,
        wheelRadiusMeters,
        measurementStdDevs
    );
    rotKinematics = new DifferentialDriveKinematics(robotWidth);

    xEncoder = new Encoder(Constants.xEncoderPortA, Constants.xEncoderPortB);
    yEncoder = new Encoder(Constants.yEncoderPortA, Constants.yEncoderPortB);
    rotationEncoder = new Encoder(Constants.rotationEncoderPortA, Constants.rotationEncoderPortB);

    xEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
    yEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
    rotationEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);

    xSimEncoder = new EncoderSim(xEncoder);
    ySimEncoder = new EncoderSim(yEncoder);
    rotationSimEncoder = new EncoderSim(rotationEncoder);

    gyro = new ADXRS450_Gyro();
    gyroSim = new ADXRS450_GyroSim(gyro);

    for(int i = 0; i < simulatedSwerveModules.length; i++) {
      simulatedSwerveModules[i] = new SwerveModuleTurnSim(
              LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerRadian,
              Constants.DriveConstants.kaVoltSecondsSquaredPerRadian),
              DCMotor.getFalcon500(1),
              Constants.ModuleConstants.kTurningMotorGearRatio,
              Constants.ModuleConstants.kWheelModuleDiameter / 2,
              null);
    }
  }

  public void setSwerveModules(SwerveModule... swerveModules) {
    m_swerveModules = swerveModules;
  }

  private void setInputs(double xVolts, double yVolts, double rotationVolts) {
    // Assume positive volts ic CW, and negative volts are CCW
    xSim.setInputs(xVolts, xVolts);
    ySim.setInputs(yVolts, yVolts);
    rotationSim.setInputs(-rotationVolts, +rotationVolts);
  }

  public void setInputState(SwerveModuleState... inputSwerveStates) {
    m_inputSwerveStates = inputSwerveStates;
  }

  /**
   * Update the drivetrain states with the current time difference.
   *
   * @param dtSeconds the time difference
   */
  @SuppressWarnings("LocalVariableName")
  public void update(double dtSeconds) {
//    simulatedChassisOutputVelocity();
    simulateCombinedSwerveModuleOutputs();
    System.out.println("x volts: " + xInputVoltage);
    System.out.println("y volts: " + yInputVoltage);
    System.out.println("r volts: " + rotInputVoltage);

    xSim.setInputs(xInputVoltage, xInputVoltage);
    xSim.update(dtSeconds);
//    xSim.setPose(new Pose2d(xDistance, yDistance, new Rotation2d(Math.PI / 2)));
    System.out.println("X L: " + xSim.getLeftPositionMeters());
    System.out.println("X R: " + xSim.getRightPositionMeters());
    xDistance = (xSim.getLeftPositionMeters() + xSim.getRightPositionMeters()) / 2;
    xVel = (xSim.getLeftVelocityMetersPerSecond() + xSim.getRightVelocityMetersPerSecond()) / 2;

    ySim.update(dtSeconds);
//    ySim.setPose(new Pose2d(xDistance, yDistance, new Rotation2d()));
    yDistance = (ySim.getLeftPositionMeters() + ySim.getRightPositionMeters()) / 2;
    yVel = (ySim.getLeftVelocityMetersPerSecond() + ySim.getRightVelocityMetersPerSecond()) / 2;

//    // Normalize diagonal speed
      // Can't really due this because it will 'desync' the swerve pose vs the sim poses.
//    double velMagnitude = Math.sqrt(Math.pow(xVel, 2) + Math.pow(yVel, 2));
//    double distanceMagnitude = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
//    if (velMagnitude > Constants.DriveConstants.kMaxSpeedMetersPerSecond) {
//      xVel /= velMagnitude;
//      yVel /= velMagnitude;
//      xDistance /= distanceMagnitude;
//      yDistance /= distanceMagnitude;
//    }

//    rotationSim.update(dtSeconds);
//    rotationSim.setPose(new Pose2d(xDistance, yDistance, rotationHeading));
    rotationHeading = rotationSim.getHeading();
//    rotationDistance = -rotationSim.getLeftPositionMeters();
//    rotationVel = -rotationSim.getLeftVelocityMetersPerSecond();
    angularVel = rotationVel / robotWidth;

    System.out.println("X Pose: " + xSim.getPose());
    System.out.println("Y Pose: " + ySim.getPose());

    System.out.println("Rot Distance: " + rotationDistance);
    System.out.println("Rot Velocity: " + angularVel);
    System.out.println("Rot Sim Heading: " + rotationSim.getHeading());
    var turnRadians = rotationDistance / (robotWidth * Math.PI);
    swervePose = new Pose2d(xDistance, yDistance, rotationHeading);
    gyroSim.setAngle(-rotationHeading.getDegrees());
//    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
//    Pose2d xPose = new Pose2d(swervePose.getTranslation(), new Rotation2d());
    //xSim.setPose(xPose);

//    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
//    Pose2d yPose = new Pose2d(swervePose.getTranslation(), new Rotation2d(Units.degreesToRadians(90)));
//    //ySim.setPose(yPose);
//
//    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
//    Pose2d rotationPose = swervePose;
//    //rotationSim.setPose(rotationPose);

  }

  /**
   * Returns the direction the robot is pointing.
   *
   * <p>Note that this angle is counterclockwise-positive, while most gyros are clockwise positive.
   */
  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  /** Returns the current pose. */
  public Pose2d getPose() {
    return swervePose;
  }

  public double getXPositionMeters() {
    return xDistance;
  }

  public double getYPositionMeters() {
    return yDistance;
  }

  public double getRotationPositionMeters() {
    return rotationDistance;
  }

  public double getXVelocityMeters() {
    return xVel;
  }

  public double getYVelocityMeters() {
      return yVel;
  }

  public double getRotationVelocityMeters() {
      return angularVel;
  }

  public double getRotationVelocityRadians() {
    return angularVel;
  }

  public Rotation2d getRotationSetpoint() {
    return new Rotation2d();
  }

  public void setFieldRelative(boolean fieldRelative) {
    isFieldRelative = fieldRelative;
  }

  /**
   * Get the drivetrain gearing.
   *
   * @return the gearing ration
   */
  public double getCurrentGearing() {
    return m_currentGearing;
  }

  /**
   * Sets the gearing reduction on the drivetrain. This is commonly used for shifting drivetrains.
   *
   * @param newGearRatio The new gear ratio, as output over input.
   */
  public void setCurrentGearing(double newGearRatio) {
    this.m_currentGearing = newGearRatio;
  }

  public void resetPose() {
    swervePose = new Pose2d();
    xSim.setPose(swervePose);
    ySim.setPose(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(90))));
    rotationSim.setPose(swervePose);
    xDistance = 0;
    yDistance = 0;
    rotationDistance = 0;
  }

  /**
   * Sets the system pose.
   *
   * @param pose The pose.
   */
  public void setPose(Pose2d pose) {
    xSim.setPose(new Pose2d(pose.getTranslation(), new Rotation2d()));
    ySim.setPose(new Pose2d(pose.getTranslation(), new Rotation2d(Units.degreesToRadians(90))));
    rotationSim.setPose(pose);
    swervePose = pose;
    xDistance = pose.getX();
    yDistance = pose.getY();
    gyroSim.setAngle(-pose.getRotation().getDegrees());
//    rotationDistance = pose.getRotation();
  }

  private final PIDController m_xPIDController = new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);
  private final PIDController m_yPIDController = new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(Constants.ModuleConstants.kPModuleTurningController, 0, 0,
          new TrapezoidProfile.Constraints(8 * Math.PI / 12, 8 * Math.PI / 12));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_xFeedforward = new SimpleMotorFeedforward(0.587, 2.3, 0.0917);
  private final SimpleMotorFeedforward m_yFeedforward = new SimpleMotorFeedforward(0.587, 2.3, 0.0917);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /*

   */

  private void simulateCombinedSwerveModuleOutputs() {
    // if the robot is not rotating, this math applies. If it is rotating, need to do something else.

    double driveMagnitude = 0;
    for(int i = 0; i < m_swerveModules.length; i++) {
      double driveInput = Math.max(-1, Math.min(m_swerveModules[i].getDriveOutput(), 1));
      driveMagnitude += driveInput;
    }
    driveMagnitude /= 4;
    System.out.println("driveMagnitude: " + driveMagnitude);

    double turnMagnitude = 0;
    for(int i = 0; i < m_swerveModules.length; i++) {
      driveMagnitude += m_swerveModules[i].getDriveOutput();
    }
    turnMagnitude /= 4;

    double cos = Units.degreesToRadians(rotationHeading.getCos());
    double sin = Units.degreesToRadians(rotationHeading.getSin());

    double xSimInput = driveMagnitude * cos + driveMagnitude * sin;
    double ySimInput = -driveMagnitude * sin + driveMagnitude * cos;

    System.out.println("Target Heading: " + rotationHeading);
    System.out.println("Cos: " + xSimInput);
    System.out.println("Sin: " + ySimInput);

//    xSimInput = xSimInput > 1 ? 1 : (xSimInput < -1 ? -1 : xSimInput);
//    ySimInput = ySimInput > 1 ? 1 : (ySimInput < -1 ? -1 : ySimInput);

    System.out.println("xInput: " + xSimInput);
    System.out.println("yInput: " + ySimInput);

    xInputVoltage = xSimInput * RobotController.getBatteryVoltage();
    yInputVoltage = ySimInput * RobotController.getBatteryVoltage();
    rotInputVoltage = 0;

    //setInputs(xInputVoltage, yInputVoltage, rotInputVoltage);
  }



  private void simulatedChassisOutputVelocity() {
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - m_lastTimestamp;

    for(int i = 0; i < 4; i++)
      m_inputSwerveStates[i] = SwerveModuleState.optimize(m_inputSwerveStates[i], getHeading());


    System.out.println("Input Angle: " + m_inputSwerveStates[0].angle.getDegrees());
    var inputChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(m_inputSwerveStates);

    System.out.println(inputChassisSpeeds);
    double xSetpoint = inputChassisSpeeds.vxMetersPerSecond;
    double ySetpoint = inputChassisSpeeds.vyMetersPerSecond;

    double angleSum = m_inputSwerveStates[0].angle.getRadians() +
                      m_inputSwerveStates[1].angle.getRadians() +
                      m_inputSwerveStates[2].angle.getRadians() +
                      m_inputSwerveStates[3].angle.getRadians();
    double angleAvg = angleSum / 4.0;
    var inputAngle = m_inputSwerveStates[0].angle.getDegrees();

    rotSetpoint = getHeading().getRadians() + inputChassisSpeeds.omegaRadiansPerSecond;
    rotSetpoint = rotSetpoint % (2 * Math.PI);

    System.out.println("Input rotation Angle: " + Units.radiansToDegrees(inputChassisSpeeds.omegaRadiansPerSecond));

//    if(Math.abs(m_lastInputAngle - inputAngle) < 0.004) {
//      rotSetpoint = m_lastRotSetpoint;
//      m_turningPIDController.setGoal(getHeading().getRadians());
//    }

//    double xFF = 0;
//    double yFF = 0;
//    double rotationFF = 0;
    double xFF = m_xFeedforward.calculate(xSetpoint);
    double yFF = m_yFeedforward.calculate(ySetpoint);
    double rotationFF = m_turnFeedforward.calculate(rotSetpoint);
//    double xFF = m_xFeedforward.calculate(xSetpoint, (xSetpoint - m_lastChassisSpeed.vxMetersPerSecond) / dt);
//    double yFF = m_yFeedforward.calculate(ySetpoint, (ySetpoint - m_lastChassisSpeed.vyMetersPerSecond) / dt);
//    double rotationFF = m_turnFeedforward.calculate(rotSetpoint, (rotSetpoint - m_lastChassisSpeed.omegaRadiansPerSecond) / dt);

    xInputVoltage = xFF + m_xPIDController.calculate(getXVelocityMeters(), xSetpoint);
    yInputVoltage = yFF + m_yPIDController.calculate(getYVelocityMeters(), ySetpoint);
//    xInputVoltage = xFF + m_xPIDController.calculate(getXVelocityMeters(), xSetpoint);
//    yInputVoltage = yFF + m_yPIDController.calculate(getYVelocityMeters(), ySetpoint);
    rotInputVoltage = rotationFF + m_turningPIDController.calculate(getHeading().getRadians(), rotSetpoint);

    System.out.println("Rotation Setpoint: " + Units.radiansToDegrees(rotSetpoint));
    System.out.println("PID Error: " + m_turningPIDController.getPositionError());
    System.out.println("Rotation Voltage: " + rotInputVoltage);

    xInputVoltage = xInputVoltage > 12 ? 12 : (xInputVoltage < -12 ? -12 : xInputVoltage);
    yInputVoltage = yInputVoltage > 12 ? 12 : (yInputVoltage < -12 ? -12 : yInputVoltage);
    rotInputVoltage = rotInputVoltage > 12 ? 12 : (rotInputVoltage < -12 ? -12 : rotInputVoltage);

    // Normalize for diagonal speed.
    // This is kinda inaccurate, but I think it works fine
    double velMagnitude = Math.sqrt(Math.pow(getXVelocityMeters(), 2) + Math.pow(getYVelocityMeters(), 2));
    double voltageMagnitude = Math.sqrt(Math.pow(xInputVoltage, 2) + Math.pow(yInputVoltage, 2));
    if (velMagnitude > Constants.DriveConstants.kMaxSpeedMetersPerSecond) {
      xInputVoltage /= voltageMagnitude;
      yInputVoltage /= voltageMagnitude;
    }

//    setInputs(xInputVoltage, yInputVoltage, rotInputVoltage);

    m_lastTimestamp = timestamp;
    m_lastChassisSpeed = inputChassisSpeeds;
    m_lastRotSetpoint = rotSetpoint;
    m_lastInputAngle = inputAngle;
  }

//  private void simulatedChassisOutputPosition() {
//    double timestamp = Timer.getFPGATimestamp();
//    double dt = timestamp - m_lastTimestamp;
//
//    System.out.println("Input State Angle 1: " + m_inputSwerveStates[0].angle);
//    System.out.println("Input State Angle 2: " + m_inputSwerveStates[1].angle);
//    System.out.println("Input State Angle 3: " + m_inputSwerveStates[2].angle);
//    System.out.println("Input State Angle 4: " + m_inputSwerveStates[3].angle);
//
////    for(int i = 0; i < 4; i++)
////      targetStates[i] = SwerveModuleState.optimize(m_inputSwerveStates[i], getHeading());
//
//    var inputChassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(m_inputSwerveStates);
//    System.out.println("Input Rad/s: " + inputChassisSpeeds.omegaRadiansPerSecond);
//    double xSetpoint = inputChassisSpeeds.vxMetersPerSecond;
//    double ySetpoint = inputChassisSpeeds.vyMetersPerSecond;
//    rotSetpoint = getHeading().getRadians() + inputChassisSpeeds.omegaRadiansPerSecond * dt;
////    rotSetpoint = rotSetpoint % (2 * Math.PI);
//
//    System.out.println("Heading Rad: " + getHeading().getRadians());
//
//    m_turningPIDController.enableContinuousInput(Math.PI, -Math.PI);
////    double xFF = 0;
////    double yFF = 0;
//    double rotationFF = 0;
////    double xFF = m_xFeedforward.calculate(xSetpoint);
////    double yFF = m_yFeedforward.calculate(ySetpoint);
////    double rotationFF = m_turnFeedforward.calculate(rotSetpoint);
//    double xFF = m_xFeedforward.calculate(xSetpoint, (xSetpoint - m_lastChassisSpeed.vxMetersPerSecond) / dt);
//    double yFF = m_yFeedforward.calculate(ySetpoint, (ySetpoint - m_lastChassisSpeed.vyMetersPerSecond) / dt);
////    double rotationFF = m_turnFeedforward.calculate(inputChassisSpeeds.omegaRadiansPerSecond, (inputChassisSpeeds.omegaRadiansPerSecond - m_lastChassisSpeed.omegaRadiansPerSecond) / dt);
//
//    xInputVoltage = xFF + m_xPIDController.calculate(getXVelocityMeters(), xSetpoint);
//    yInputVoltage = yFF + m_yPIDController.calculate(getYVelocityMeters(), ySetpoint);
//    rotInputVoltage = rotationFF + m_turningPIDController.calculate(getHeading().getRadians(), rotSetpoint);
//
//
//
//    System.out.println("Sim Pose: " + getPose());
////    System.out.println("X FF: " + xFF);
////    System.out.println("X PID: " + (xInputVoltage - xFF));
////    System.out.println("Setpoint X: " + xSetpoint);
////    System.out.println("Setpoint Y: " + ySetpoint);
//    System.out.println("Setpoint R: " + rotSetpoint);
////    System.out.println("Input Volts X: " + xInputVoltage);
////    System.out.println("Input Volts Y: " + yInputVoltage);
//    System.out.println("Input Volts R: " + rotInputVoltage);
//
//    setInputs(xInputVoltage, yInputVoltage, rotInputVoltage);
//
//    m_lastTimestamp = timestamp;
//    m_lastChassisSpeed = inputChassisSpeeds;
//    m_lastRotSetpoint = rotSetpoint;
//  }
}