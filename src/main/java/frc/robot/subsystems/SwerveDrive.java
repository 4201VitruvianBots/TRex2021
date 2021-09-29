/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {
    static double kP = 0.005;
    static double kI = 0;
    static double kD = 0.00025;

    /**
     * Just like a graph's quadrants
     * 0 is Front Left
     * 1 is Back Left
     * 2 is Front Right
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
            new SwerveModule(0, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), new CANCoder(Constants.frontLeftCANCoder), 106.436, true, false),//-267.715
            new SwerveModule(1, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), new CANCoder(Constants.frontRightCANCoder), 219.639, true, false), //true -266.572
            new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), new CANCoder(Constants.backLeftCANCoder), 98.877, true, false),//-154.600
            new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), new CANCoder(Constants.backRightCANCoder), 70, true, false) //true -166.816
    };

    PowerDistributionPanel m_pdp;

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);
    int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getRotation());
//    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
//        getRotation(),
//        new Pose2d(),
//        kDriveKinematics,
//        VecBuilder.fill(0.1, 0.1, 0.1),
//        VecBuilder.fill(0.05),
//        VecBuilder.fill(0.1, 0.1, 0.1));

    private ProfiledPIDController turnPidController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(360, 360));

    private boolean enablePidTurn = false;
    private double pidTurnOutput;

    private Rotation2d headingTarget;
    private Pose2d headingTargetPosition = new Pose2d(-1, -1, new Rotation2d());

    private double m_trajectoryTime;
    private Trajectory currentTrajectory;

    public SwerveDrive(PowerDistributionPanel pdp) {
        m_pdp = pdp;

        turnPidController.setTolerance(0.2);
        turnPidController.enableContinuousInput(-180, 180);

        SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this);
        if (RobotBase.isSimulation()) {

        }
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
//        if(RobotBase.isReal())
//            return Rotation2d.fromDegrees(getHeading());
//        else
//            try {
//                return swerveChassisSim.getHeading();
//            } catch (Exception e) {
//                return new Rotation2d();
//            }
        return Rotation2d.fromDegrees(getHeadingDegrees());
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
    public double getHeadingDegrees() {
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++)
            mSwerveModules[i].resetEncoders();
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

    SlewRateLimiter xLimit = new SlewRateLimiter(kMaxSpeedMetersPerSecond * 10);
    SlewRateLimiter yLimit = new SlewRateLimiter(kMaxSpeedMetersPerSecond * 10);
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        xSpeed *= kMaxSpeedMetersPerSecond;
        ySpeed *= kMaxSpeedMetersPerSecond;
        rot *= kMaxChassisRotationSpeed;

        // xSpeed = xLimit.calculate(xSpeed);
        // ySpeed = yLimit.calculate(ySpeed);

        //If pidTurn is getting a value override the drivers steering control
        if (enablePidTurn) {
            rot = turnPidController.calculate(getHeadingDegrees());
        }

        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        ); //from 2910's code
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

        mSwerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
        mSwerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
        mSwerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);
        mSwerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
    }

    public void setAngleSetpoint(double angleSetpoint, boolean enabled) {
        enablePidTurn = enabled;
//        double differance = angle - RobotContainer.swerve.getDegrees();
//        if (differance > 180) {
//            differance = (360 - differance) * -1;
//        }
        turnPidController.setGoal(angleSetpoint - getHeadingDegrees());
    }

    public void setSwerveDriveNeutralMode(boolean mode) {
        for(int i = 0; i < mSwerveModules.length; i++)
            mSwerveModules[i].setBrakeMode(mode);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        mSwerveModules[0].setDesiredState(desiredStates[0], false);
        mSwerveModules[1].setDesiredState(desiredStates[1], false);
        mSwerveModules[2].setDesiredState(desiredStates[2], false);
        mSwerveModules[3].setDesiredState(desiredStates[3], false);
    }

    public void setHeadingToTargetHeading(Rotation2d targetHeading) {
        headingTarget = targetHeading;
    }

    public void setHeadingToTargetPosition(Pose2d targetPosition) {
        headingTargetPosition = targetPosition;
    }

    public Rotation2d getHeadingToTarget() {
        return headingTarget;
    }

    public Pose2d getTargetPose() {
        return headingTargetPosition;
    }
    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        m_odometry.update(
            getRotation(),
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        );
        // Update module positions based on the chassis' position, but keep the module heading
        for (int i = 0; i < mSwerveModules.length; i++) {
            var modulePositionFromChassis = modulePositions[i].rotateBy(getRotation()).plus(getPose().getTranslation());
            mSwerveModules[i].setPose(new Pose2d(modulePositionFromChassis, mSwerveModules[i].getHeading().plus(getRotation())));
        }

//        m_odometry.addVisionMeasurement(SimulationReferencePose.getRobotFieldPose(), Timer.getFPGATimestamp() + 0.2);

//        System.out.println("Swerve Pose: " + getPose());
//        System.out.println("Field Pose: " + SimulationReferencePose.getRobotFieldPose());
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        m_odometry.resetPosition(pose, rotation);

        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setPose(pose);
            mSwerveModules[i].resetEncoders();
        }
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("SwerveDrive","Chassis Angle", getHeadingDegrees());
        for(int i = 0; i < mSwerveModules.length; i++) {
            SmartDashboardTab.putNumber("SwerveDrive", "Swerve Module " + i + " Angle", mSwerveModules[i].getState().angle.getDegrees());
            SmartDashboardTab.putNumber("SwerveDrive", "Swerve Module " + i + " Speed", mSwerveModules[i].getState().speedMetersPerSecond);
        }


//        SmartDashboardTab.putNumber("SwerveDrive","Front Left Angle",mSwerveModules[0].getTurnAngle());
//        SmartDashboardTab.putNumber("SwerveDrive","Back Left Angle",mSwerveModules[1].getTurnAngle());
//        SmartDashboardTab.putNumber("SwerveDrive","Front Right Angle",mSwerveModules[2].getTurnAngle());
//        SmartDashboardTab.putNumber("SwerveDrive","Back Right Angle",mSwerveModules[3].getTurnAngle());
//
//        SmartDashboardTab.putNumber("SwerveDrive","navXDebug",navXDebug);
//        SmartDashboardTab.putNumber("SwerveDrive","State",mSwerveModules[0].getState().angle.getDegrees());

        SmartDashboardTab.putNumber("SwerveDrive", "X coordinate", getPose().getX());
        SmartDashboardTab.putNumber("SwerveDrive", "Y coordinate", getPose().getY());
//    SmartDashboardTab.putNumber("SwerveDrive","Front Right Speed",mSwerveModules[0].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Front Left Speed",mSwerveModules[1].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Left Speed",mSwerveModules[2].getState().speedMetersPerSecond);
//    SmartDashboardTab.putNumber("SwerveDrive","Back Right Speed",mSwerveModules[3].getState().speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        sampleTrajectory();
        updateOdometry();
        updateSmartDashboard();

        setHeadingToTargetPosition(new Pose2d(4.5, 4, new Rotation2d()));
        if(headingTargetPosition.getX() != -1 && headingTargetPosition.getY() != -1) {
            double yDelta = headingTargetPosition.getY() - getPose().getY();
            double xDelta = headingTargetPosition.getX() - getPose().getX();
            var target = new Rotation2d(Math.atan2(yDelta, xDelta));

//            if(inputTurnInversion == -1)
//                target = target.unaryMinus();

            setHeadingToTargetHeading(target);
            //System.out.println("Target Heading: " + getHeadingToTarget());
        }

        // This method will be called once per scheduler run
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = {
            mSwerveModules[0].getPose(),
            mSwerveModules[1].getPose(),
            mSwerveModules[2].getPose(),
            mSwerveModules[3].getPose()
        };
        return modulePoses;
    }

    double yaw = 0;
    @Override
    public void simulationPeriodic() {
        SwerveModuleState[] moduleStates = {
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        };

        var chassisSpeed = kDriveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        yaw += chassisRotationSpeed * 0.02;
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
//        angle.set(Math.IEEEremainder(-swerveChassisSim.getHeading().getDegrees(), 360));
        angle.set(-Units.radiansToDegrees(yaw));
    }

    private void sampleTrajectory() {
        if(DriverStation.getInstance().isAutonomous()) {
            try {
                var currentTrajectoryState = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime);

                System.out.println("Trajectory Time: " + (Timer.getFPGATimestamp() - startTime));
                System.out.println("Trajectory Pose: " + currentTrajectoryState.poseMeters);
                System.out.println("Trajectory Speed: " + currentTrajectoryState.velocityMetersPerSecond);
                System.out.println("Trajectory angular speed: " + currentTrajectoryState.curvatureRadPerMeter);
            } catch (Exception e) {

            }
        }

    }

    public void setTrajectoryTime(double trajectoryTime) {
        m_trajectoryTime = trajectoryTime;
    }

    double startTime;
    public void setCurrentTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        startTime = Timer.getFPGATimestamp();
    }
}
