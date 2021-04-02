/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SwerveModuleSim;

import static frc.robot.Constants.DriveConstants.kDriveKinematics;

public class SwerveDrive extends SubsystemBase {

    public static final double kMaxSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond; // 3 meters per second
    public static final double kMaxAngularSpeed = Constants.ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond; // 3 meters per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private int navXDebug = 0;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getRotation());

    private SwerveModuleSim swerveModuleSim;
    private double inputRotationSpeed;

    PowerDistributionPanel m_pdp;

    private double m_trajectoryTime;
    private Trajectory currentTrajectory;

    private Rotation2d headingTarget;
    private Pose2d headingTargetPosition = new Pose2d(-1, -1, new Rotation2d());

    /**
     * Just like a graph's quadrants
     * 0 is Front Left
     * 1 is Back Left
     * 2 is Front Right
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
            new SwerveModule(0, new TalonFX(Constants.frontLeftTurningMotor), new TalonFX(Constants.frontLeftDriveMotor), 0, true, false),
            new SwerveModule(1, new TalonFX(Constants.frontRightTurningMotor), new TalonFX(Constants.frontRightDriveMotor), 0, true, false), //true
            new SwerveModule(2, new TalonFX(Constants.backLeftTurningMotor), new TalonFX(Constants.backLeftDriveMotor), 0, true, false),
            new SwerveModule(3, new TalonFX(Constants.backRightTurningMotor), new TalonFX(Constants.backRightDriveMotor), 0, true, true) //true
    };

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);
    int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    public SwerveDrive(PowerDistributionPanel pdp) {
        m_pdp = pdp;

        SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this);
        if (RobotBase.isSimulation()) {
            swerveModuleSim = new SwerveModuleSim(kDriveKinematics, mSwerveModules);
//            simulateSwerveDrive = new SimulateSwerveDrive(
//                    Constants.DriveConstants.kDrivetrainPlant,
//                    Constants.DriveConstants.kDriveGearbox,
//                    Constants.ModuleConstants.kDriveMotorGearRatio,
//                    Constants.DriveConstants.kTrackWidth,
//                    Constants.DriveConstants.kWheelBase,
//                    Constants.ModuleConstants.kWheelDiameterMeters / 2.0,
//                    null
//            );
//            simulateSwerveDrive.setSwerveModules(mSwerveModules);
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
        return Rotation2d.fromDegrees(getHeading());
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
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }
    public Rotation2d getSimulatedHeading() {
        return swerveModuleSim.getChassisHeading();
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
        xSpeed *= kMaxSpeed;
        ySpeed *= kMaxSpeed;
        rot *= kMaxAngularSpeed;
        inputRotationSpeed = rot;

        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        ); //from 2910's code
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

        // I'm not exactly sure why this needs to be inverted in teleop, but not auto?
        if(RobotBase.isSimulation())
            for(int i = 0; i < mSwerveModules.length; i++)
                swerveModuleStates[i].angle = swerveModuleStates[i].angle.unaryMinus();


        mSwerveModules[0].setDesiredState(swerveModuleStates[0]);
        mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
        mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
        mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    public void setSwerveDriveNeutralMode(boolean mode) {
        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setBrakeMode(mode);
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        var chassisSpeed = kDriveKinematics.toChassisSpeeds(desiredStates);
        inputRotationSpeed =  chassisSpeed.omegaRadiansPerSecond;
        System.out.println("Chassis Rotation: " + chassisSpeed.omegaRadiansPerSecond);
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        // mSwerveModules[0].setDesiredState(desiredStates[0]);
        // mSwerveModules[2].setDesiredState(desiredStates[1]);
        // mSwerveModules[1].setDesiredState(desiredStates[2]);
        // mSwerveModules[3].setDesiredState(desiredStates[3]);
        mSwerveModules[0].setDesiredState(desiredStates[0]);
        mSwerveModules[1].setDesiredState(desiredStates[1]);
        mSwerveModules[2].setDesiredState(desiredStates[2]);
        mSwerveModules[3].setDesiredState(desiredStates[3]);
    }

    public void setVoltageOutput(double leftVoltage, double rightVoltage) {
        double batteryVoltage = RobotController.getBatteryVoltage();
        mSwerveModules[0].setPercentOutput(leftVoltage / batteryVoltage);
        mSwerveModules[2].setPercentOutput(leftVoltage / batteryVoltage);
        mSwerveModules[1].setPercentOutput(rightVoltage / batteryVoltage);
        mSwerveModules[3].setPercentOutput(rightVoltage / batteryVoltage);
    }

    public void setSimPoses(Pose2d robotPose, Rotation2d[] headings){
        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setSimulatedState(new SwerveModuleState(0, headings[i]));
        }

        swerveModuleSim.setPosesFromChassis(robotPose, headings);
    }

    public Pose2d[] getSimPoses(){
        return swerveModuleSim.getSimPoses();
    }

    public void setHeadingToTargetHeading(Rotation2d targetHeading) {
        headingTargetPosition = new Pose2d(-1, -1, new Rotation2d());
        headingTarget = targetHeading;
    }

    public void setHeadingToTargetPosition(Pose2d targetPosition) {
        headingTargetPosition = targetPosition;
    }

    public Rotation2d getHeadingTarget() {
        return headingTarget;
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
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        m_odometry.resetPosition(pose, rotation);
        swerveModuleSim.setPosesFromChassis(pose);
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("SwerveDrive","Chassis Angle",getHeading());
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

        setHeadingToTargetPosition(new Pose2d(8, 8, new Rotation2d()));
        if(headingTargetPosition.getX() != -1 && headingTargetPosition.getY() != -1) {
            double yDelta = headingTargetPosition.getY() - getPose().getY();
            double xDelta = headingTargetPosition.getX() - getPose().getX();
            setHeadingToTargetHeading(new Rotation2d(Math.atan2(yDelta, xDelta)));
            System.out.println("Target Heading: " + getHeadingTarget());
        }

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

        swerveModuleSim.setInputRotationSpeed(inputRotationSpeed);
        swerveModuleSim.update(0.02);

        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
        angle.set(Math.IEEEremainder(-swerveModuleSim.getChassisHeading().getDegrees(), 360));

        // NavX expects clockwise positive, but sim outputs clockwise negative
//        simulateSwerveDrive.setFieldRelative(isFieldOriented);
//        simulateSwerveDrive.setInputState(simulationSwerveModuleStates);
//        simulateSwerveDrive.update(0.02);
//        for (int i = 0; i < 4; i++) {
//            mSwerveModules[i].setTurnEncoderSimAngle(simulationSwerveModuleStates[i].angle.getDegrees());
//            mSwerveModules[i].setTurnEncoderSimRate(simulationSwerveModuleStates[i].speedMetersPerSecond);
//        }
    }

    private void sampleTrajectory() {
        if(DriverStation.getInstance().isAutonomous()) {
            try {
                var currentTrajectoryState = currentTrajectory.sample(m_trajectoryTime);
                System.out.println("TrajectoryTime: " + m_trajectoryTime);
                System.out.println("TrajectoryPose: " + currentTrajectoryState.poseMeters);
            } catch (Exception e) {

            }
        }

    }

    public void setTrajectoryTime(double trajectoryTime) {
        m_trajectoryTime = trajectoryTime;
    }
    public void setCurrentTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
    }


}
