/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {


  public int controlMode = 0;
  
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private boolean isFieldOriented;
  private final double throttle = 0.8;
  private final double turningThrottle = 0.5;
  
  // ???
  private double kS = 0.19;
  private double kV = 2.23;
  private double kA = 0.0289;

  // PID controller values
  public double kP = 1.33;
  public double kI = 0;
  public double kD = 0; 
    
  // Set up spatial tools
 DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
 private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getAngle());

 SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

 // Sets up PIDcontroller
 PIDController leftPIDController = new PIDController(kP, kI, kD);
 PIDController rightPIDController = new PIDController(kP, kI, kD);
  PowerDistributionPanel m_pdp;
  /**
   * Just like a graph's quadrants
   * 0 is Front Right
   * 1 is Front Left
   * 2 is Back Left
   * 3 is Back Right
   */
  private final SwerveModule[] mSwerveModules = new SwerveModule[] {
          new SwerveModule(0, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), 0, false, m_pdp), //true
          new SwerveModule(1, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), 0, false, m_pdp),
          new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), 0, false, m_pdp),
          new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), 0, false, m_pdp) //true
  };

  private final AHRS mNavX = new AHRS(SerialPort.Port.kMXP);

  public void resetOdometry(final Pose2d pose, final Rotation2d rotation) {
    m_odometry.resetPosition(pose, rotation);
  }

  public SwerveDrive(final PowerDistributionPanel pdp) {
    m_pdp = pdp;

    mNavX.reset();
  }

  public AHRS getNavX() {
    return mNavX;
  }

  public double getGyroRate() {
    return mNavX.getRate();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(getRawGyroAngle());
  }

  public double getRawGyroAngle() {
    return mNavX.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return mNavX.getRate();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(mNavX.getAngle(), 360);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    for (int i = 0; i < 4; i++){
      mSwerveModules[i].resetEncoders();
    }
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    mNavX.reset();
  }


  public SwerveModule getSwerveModule(final int i) {
    return mSwerveModules[i];
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getSwerveDriveKinematics() {
    return kinematics;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(final double xSpeed, final double ySpeed, final double rot, final boolean fieldRelative) {
    final var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, getAngle())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    mSwerveModules[1].setDesiredState(swerveModuleStates[0]);
    mSwerveModules[0].setDesiredState(swerveModuleStates[1]);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(final SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    mSwerveModules[1].setDesiredState(desiredStates[0]);
    mSwerveModules[0].setDesiredState(desiredStates[1]);
    mSwerveModules[2].setDesiredState(desiredStates[2]);
    mSwerveModules[3].setDesiredState(desiredStates[3]);
  }

//  public void holonomicDrive(double forward, double strafe, double rotationSpeed) {
//    forward *= throttle; //because if they are both 1, then max output is sqrt(2), which is more than 1.
//    strafe *= throttle;
//    //todo: rotationSpeed += PIDOutput //this PID calculates the speed needed to turn to a setpoint based off of a button input. Probably from the D-PAD
//    rotationSpeed *= turningThrottle; //I'll also have to check to make sure this isn't too high.
//    if (isFieldOriented) { //checks to see if it's field oriented
//      double angleRad = Math.toRadians(getRawGyroAngle());
//      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad); //calculates new forward
//      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad); //calculates new strafe
//      forward = temp;
//    }
//
//    double a = strafe - rotationSpeed * (WHEELBASE / 2); //calculations from document
//    double b = strafe + rotationSpeed * (WHEELBASE / 2);
//    double c = forward - rotationSpeed * (TRACKWIDTH / 2);
//    double d = forward + rotationSpeed * (TRACKWIDTH / 2);
//
//    double[] angles = new double[]{ //calculates the angle needed for each module
//            Math.atan2(b, c) * 180 / Math.PI,
//            Math.atan2(b, d) * 180 / Math.PI,
//            Math.atan2(a, d) * 180 / Math.PI,
//            Math.atan2(a, c) * 180 / Math.PI
//    };
//
//    double[] speeds = new double[]{ //calculates the speed needed for each module
//            Math.sqrt(b * b + c * c),
//            Math.sqrt(b * b + d * d),
//            Math.sqrt(a * a + d * d),
//            Math.sqrt(a * a + c * c)
//    };
//
//    double max = speeds[0];
//
//    for (double speed : speeds) {
//      if (speed > max) {
//        max = speed; //looks for the max
//      }
//    }
//
//    if(max > 1) { //this makes sure that no speed is greater than 1.
//      for (int i = 0; i < 4; i++){
//        speeds[i] /= max; //if one is, scale them all down by the max.
//      }
//    }
//
//    for (int i = 0; i < 4; i++) {
//      if (Math.abs(forward) > 0.05 ||
//              Math.abs(strafe) > 0.05 ||
//              Math.abs(rotationSpeed) > 0.05) {
//        mSwerveModules[i].setTargetAngle(angles[i] + 180); //to get it within 0 to 360. It was in -180 to 180
//      } else {
//        mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
//      }
//      mSwerveModules[i].setPercentOutput(speeds[i]);
//    }
//  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
            getAngle(),
            mSwerveModules[1].getState(),
            mSwerveModules[0].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
    );
  }

  private void updateSmartDashboard() {
    SmartDashboardTab.putNumber("SwerveDrive","Angle",getRawGyroAngle());
    SmartDashboardTab.putNumber("SwerveDrive","Front Right Angle",mSwerveModules[0].getState().angle.getDegrees());
    SmartDashboardTab.putNumber("SwerveDrive","Front Left Angle",mSwerveModules[1].getState().angle.getDegrees());
    SmartDashboardTab.putNumber("SwerveDrive","Back Left Angle",mSwerveModules[2].getState().angle.getDegrees());
    SmartDashboardTab.putNumber("SwerveDrive","Back Right Angle",mSwerveModules[3].getState().angle.getDegrees());
    SmartDashboardTab.putNumber("SwerveDrive","Front Right Speed",mSwerveModules[0].getState().speedMetersPerSecond);
    SmartDashboardTab.putNumber("SwerveDrive","Front Left Speed",mSwerveModules[1].getState().speedMetersPerSecond);
    SmartDashboardTab.putNumber("SwerveDrive","Back Left Speed",mSwerveModules[2].getState().speedMetersPerSecond);
    SmartDashboardTab.putNumber("SwerveDrive","Back Right Speed",mSwerveModules[3].getState().speedMetersPerSecond);
  }

  @Override
  public void periodic() {
    updateOdometry();

    // This method will be called once per scheduler run
  }

}
