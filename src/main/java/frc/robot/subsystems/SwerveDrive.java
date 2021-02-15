/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SimulateSwerveDrive;

public class SwerveDrive extends SubsystemBase {

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private int navXDebug = 0;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.DriveConstants.kDriveKinematics, getRotation());

    private double xInput, yInput, rotationInput;

    private Encoder xEncoder, yEncoder, rotationEncoder;
    private EncoderSim xSimEncoder, ySimEncoder, rotationSimEncoder;
    private ADXRS450_Gyro gyro;
    private ADXRS450_GyroSim gyroSim;

    private SimulateSwerveDrive simulateSwerveDrive;

    private boolean gyroWasNull = false;

    PowerDistributionPanel m_pdp;
    /**
     * Just like a graph's quadrants
     * 0 is Front Left
     * 1 is Back Left
     * 2 is Front Right
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
            new SwerveModule(0, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), 0, true, false),
            new SwerveModule(1, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), 0, true, false),
            new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), 0, true, false),
            new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), 0, true, false)
    };

    private SwerveModuleState[] simulationSwerveModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);

    public void testTurningMotor(double speed){
        mSwerveModules[0].mTurningMotor.set(ControlMode.PercentOutput,speed);
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        if (RobotBase.isSimulation()) {
            resetEncoders();
            simulateSwerveDrive.setPose(pose);
        }
        m_odometry.resetPosition(pose, rotation);
    }

    public SwerveDrive(PowerDistributionPanel pdp) {
        m_pdp = pdp;

        SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this);
        if (RobotBase.isSimulation()) {
            xEncoder = new Encoder(Constants.xEncoderPortA, Constants.xEncoderPortB);
            yEncoder = new Encoder(Constants.yEncoderPortA, Constants.yEncoderPortB);
            rotationEncoder = new Encoder(Constants.rotationEncoderPortA, Constants.rotationEncoderPortB);

            xSimEncoder = new EncoderSim(xEncoder);
            ySimEncoder = new EncoderSim(yEncoder);
            rotationSimEncoder = new EncoderSim(rotationEncoder);

            xEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
            yEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
            rotationEncoder.setDistancePerPulse(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);

            gyro = new ADXRS450_Gyro();
            gyroSim = new ADXRS450_GyroSim(gyro);

            simulateSwerveDrive = new SimulateSwerveDrive(
                    Constants.DriveConstants.kDrivetrainPlant,
                    Constants.DriveConstants.kDriveGearbox,
                    Constants.ModuleConstants.kDriveMotorGearRatio,
                    Constants.DriveConstants.kTrackWidth,
                    Constants.DriveConstants.kWheelBase,
                    Constants.ModuleConstants.kWheelDiameterMeters / 2.0,
                    null
            );
        }
    }

    /**
     * Returns the raw angle of the robot in degrees
     *
     * @return The angle of the robot
     */
    public double getRawGyroAngle() {
        if (RobotBase.isReal()) {
            try {
                return mNavX.getAngle();
            } catch (Exception e) {
                navXDebug = 1;
                return 0;
            }
        } else {
            try {
                return gyro.getAngle();
            } catch (Exception e) {
                return 0;
            }
        }
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
    public Rotation2d getRotation() {
        // Negating the angle because WPILib gyros are CW positive.
        return Rotation2d.fromDegrees(getRawGyroAngle());
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
        if (RobotBase.isReal()) {
            return Math.IEEEremainder(mNavX.getAngle(), 360);
        } else {
            return Math.IEEEremainder(gyro.getAngle(), 360);
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++){
            mSwerveModules[i].resetEncoders();
        }
        if (RobotBase.isSimulation()) {
        }
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        mNavX.reset();
    }


    public SwerveModule getSwerveModule(int i) {
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

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (Math.abs(xSpeed)<=0.05)
            xSpeed=0;
        if (Math.abs(ySpeed)<=0.05)
            ySpeed=0;
        if (Math.abs(rot)<=0.05)
            rot=0;
        xInput = xSpeed;
        yInput = ySpeed;
        rotationInput = rot;
        var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        ); //from 2910's code
        //todo: rotationSpeed += PIDOutput //this PID calculates the speed needed to turn to a setpoint based off of a button input. Probably from the D-PAD
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        SmartDashboardTab.putNumber("SwerveDrive","Desired State",swerveModuleStates[0].angle.getDegrees());
        mSwerveModules[0].setDesiredState(swerveModuleStates[0]);
        mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
        mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
        mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        mSwerveModules[0].setDesiredState(desiredStates[0]);
        mSwerveModules[1].setDesiredState(desiredStates[1]);
        mSwerveModules[2].setDesiredState(desiredStates[2]);
        mSwerveModules[3].setDesiredState(desiredStates[3]);

        ChassisSpeeds speeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
        Rotation2d robotAngle = getRotation();

        // Convert from robot-relative to field-relative
        xInput = speeds.vxMetersPerSecond * robotAngle.getCos() - speeds.vyMetersPerSecond * robotAngle.getSin();
        yInput = speeds.vxMetersPerSecond * robotAngle.getSin() + speeds.vyMetersPerSecond * robotAngle.getCos();
        rotationInput = speeds.omegaRadiansPerSecond;

        // Normalization
        xInput /= Constants.AutoConstants.kMaxSpeedMetersPerSecond;
        yInput /= Constants.AutoConstants.kMaxSpeedMetersPerSecond;
        rotationInput /= Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;

        double magnitude = Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2));
        if (magnitude > 1) {
            xInput /= magnitude;
            yInput /= magnitude;
        }
    }

    public void setSwerveDriveNeutralMode(boolean mode) {
        mSwerveModules[0].setBrakeMode(mode);
        mSwerveModules[1].setBrakeMode(mode);
        mSwerveModules[2].setBrakeMode(mode);
        mSwerveModules[3].setBrakeMode(mode);
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
        if (RobotBase.isReal()) {
            m_odometry.update(
                    getRotation(),
                    mSwerveModules[0].getState(),
                    mSwerveModules[1].getState(),
                    mSwerveModules[2].getState(),
                    mSwerveModules[3].getState()
            );
        } else {
            m_odometry.update(
                    getRotation(),
                    simulationSwerveModuleStates[0],
                    simulationSwerveModuleStates[1],
                    simulationSwerveModuleStates[2],
                    simulationSwerveModuleStates[3]
            );
        }
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("SwerveDrive","Angle",getRawGyroAngle());
        SmartDashboardTab.putNumber("SwerveDrive","Front Left Angle",mSwerveModules[0].getTurnAngle());
        SmartDashboardTab.putNumber("SwerveDrive","Back Left Angle",mSwerveModules[1].getTurnAngle());
        SmartDashboardTab.putNumber("SwerveDrive","Front Right Angle",mSwerveModules[2].getTurnAngle());
        SmartDashboardTab.putNumber("SwerveDrive","Back Right Angle",mSwerveModules[3].getTurnAngle());

        SmartDashboardTab.putNumber("SwerveDrive","navXDebug",navXDebug);
        SmartDashboardTab.putNumber("SwerveDrive","State",mSwerveModules[0].getState().angle.getDegrees());

        SmartDashboardTab.putNumber("SwerveDrive", "X coordinate", getPose().getX());
        SmartDashboardTab.putNumber("SwerveDrive", "Y coordinate", getPose().getY());
//    SmartDashboardTab.putNumber("SwerveDrive","Front Right Speed",mSwerveModules[0].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Front Left Speed",mSwerveModules[1].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Left Speed",mSwerveModules[2].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Right Speed",mSwerveModules[3].getState().speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateSmartDashboard();

        // This method will be called once per scheduler run
    }

    public Rotation2d [] getModuleHeadings() {
        Rotation2d [] modulePositions = {
            mSwerveModules[0].getHeading(),
            mSwerveModules[1].getHeading(),
            mSwerveModules[2].getHeading(),
            mSwerveModules[3].getHeading()
        };
        return modulePositions;
    }

    @Override
    public void simulationPeriodic() {
        simulateSwerveDrive.setInputs(xInput * RobotController.getBatteryVoltage(),
                yInput * RobotController.getBatteryVoltage(),
                rotationInput * RobotController.getBatteryVoltage());
        simulateSwerveDrive.update(0.020);
        

        xSimEncoder.setDistance(simulateSwerveDrive.getXPositionMeters());
        ySimEncoder.setDistance(simulateSwerveDrive.getYPositionMeters());
        rotationSimEncoder.setDistance(simulateSwerveDrive.getRotationPositionMeters());

        xSimEncoder.setRate(simulateSwerveDrive.getXVelocityMeters());
        ySimEncoder.setRate(simulateSwerveDrive.getYVelocityMeters());
        rotationSimEncoder.setRate(simulateSwerveDrive.getRotationVelocityMeters());

        gyroSim.setAngle(-simulateSwerveDrive.getHeading().getDegrees());

        var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xEncoder.getRate(), yEncoder.getRate(), rotationEncoder.getRate(), getRotation())
        );
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        simulationSwerveModuleStates = swerveModuleStates;
        setModuleStates(swerveModuleStates);
        for (int i = 0; i < 4; i++) {
            mSwerveModules[i].setTurnEncoderSimAngle(simulationSwerveModuleStates[i].angle.getDegrees());
            mSwerveModules[i].setTurnEncoderSimRate(simulationSwerveModuleStates[i].speedMetersPerSecond);
        }
    }
}
