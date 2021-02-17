package frc.robot.simulation;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N7;
import frc.robot.Constants;

public class SimulateSwerveDrive {
    private final DCMotor m_motor;
  private final double m_originalGearing;
  private final Matrix<N7, N1> m_measurementStdDevs;
  private double m_currentGearing;
  private final double m_wheelRadiusMeters;

  private final LinearSystem<N2, N2, N2> m_plant;

  private final double robotWidth, robotLength;

  private DifferentialDrivetrainSim xSim, ySim, rotationSim;
  private double xDistance, yDistance, rotationDistance;
  private double xVel, yVel, angularVel;
  private Pose2d swervePose;

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
    ySim = new DifferentialDrivetrainSim(
        drivetrainPlant,
        driveMotor,
        gearing,
        robotLength,
        wheelRadiusMeters,
        measurementStdDevs
    );
    ySim.setPose(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(90))));
    rotationSim = new DifferentialDrivetrainSim(
        drivetrainPlant,
        driveMotor,
        gearing,
        robotWidth,
        wheelRadiusMeters,
        measurementStdDevs
    );
  }

  public void setInputs(double xVolts, double yVolts, double rotationVolts) {
    // Assume positive volts ic CW, and negative volts are CCW
    xSim.setInputs(xVolts, -xVolts);
    ySim.setInputs(yVolts, -yVolts);
    rotationSim.setInputs(rotationVolts, rotationVolts);
  }

  /**
   * Update the drivetrain states with the current time difference.
   *
   * @param dtSeconds the time difference
   */
  @SuppressWarnings("LocalVariableName")
  public void update(double dtSeconds) {
    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
    Pose2d xPose = new Pose2d(swervePose.getTranslation(), new Rotation2d());
    xDistance += (xSim.getLeftPositionMeters() - xSim.getRightPositionMeters()) / 2;
    xVel = (xSim.getLeftVelocityMetersPerSecond() - xSim.getRightVelocityMetersPerSecond()) / 2;
    xSim.setPose(xPose);
    xSim.update(dtSeconds);

    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
    Pose2d yPose = new Pose2d(swervePose.getTranslation(), new Rotation2d(Units.degreesToRadians(90)));
    yDistance += (ySim.getLeftPositionMeters() - ySim.getRightPositionMeters()) / 2;
    yVel = (ySim.getLeftVelocityMetersPerSecond() - ySim.getRightVelocityMetersPerSecond()) / 2;
    ySim.setPose(yPose);
    ySim.update(dtSeconds);

    swervePose = new Pose2d(xSim.getPose().getX(), ySim.getPose().getY(), rotationSim.getHeading()); // Keeps track of robot's position
    Pose2d rotationPose = swervePose;
    rotationDistance += (rotationSim.getLeftPositionMeters() + rotationSim.getRightPositionMeters()) / 2;
    angularVel = (rotationSim.getLeftVelocityMetersPerSecond() + rotationSim.getRightVelocityMetersPerSecond()) / 2;
    rotationSim.setPose(rotationPose);
    rotationSim.update(dtSeconds);

    SmartDashboardTab.putNumber("SwerveDrive", "X distance", xDistance);
    SmartDashboardTab.putNumber("SwerveDrive", "simulator x coordinate", xSim.getPose().getX());

    SmartDashboardTab.putNumber("SwerveDrive", "Y distance", yDistance);
    SmartDashboardTab.putNumber("SwerveDrive", "simulator y coordinate", swervePose.getY());

    SmartDashboardTab.putNumber("SwerveDrive", "x vel", xVel);
  }

  /**
   * Returns the direction the robot is pointing.
   *
   * <p>Note that this angle is counterclockwise-positive, while most gyros are clockwise positive.
   */
  public Rotation2d getHeading() {
    return swervePose.getRotation();
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
//    xDistance = 0;
//    yDistance = 0;
//    rotationDistance = 0;
  }
}