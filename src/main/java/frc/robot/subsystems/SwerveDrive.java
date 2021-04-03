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
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.simulation.SimulationReferencePose;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

    public static final double kMaxSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond; // 3 meters per second
    public static final double kMaxAngularSpeed = kMaxChassisRotationSpeed; // 3 meters per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private int navXDebug = 0;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kDriveKinematics, getRotation());
//    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
//        getRotation(),
//        new Pose2d(),
//        kDriveKinematics,
//        VecBuilder.fill(0.1, 0.1, 0.1),
//        VecBuilder.fill(0.05),
//        VecBuilder.fill(0.1, 0.1, 0.1));

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

    private DifferentialDrivetrainSim swerveChassisSim = new DifferentialDrivetrainSim(
        Constants.DriveConstants.kDrivetrainPlant,
        Constants.DriveConstants.kDriveGearbox,
        Constants.ModuleConstants.kDriveMotorGearRatio,
        Constants.DriveConstants.kTrackWidth,
        Constants.ModuleConstants.kWheelDiameterMeters / 2.0,
        null
    );

    private SwerveModuleState[] inputStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private double inputRotationSpeed;
    // Run the calculation for SimChassisInputVoltage without the fudge and print out the rotation input from a controller
    // and the outputted sim rotation angular velocity (Velocity of one side of the drivetrain / tradkwidth / 2). The ratio
    // between these two numbers is the fudge in order to get the rotation correct.
    private double inputRotationFudge = 0.850330956556336;

    private double inputTurnInversion;
    private double lastInputSign;

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);
    int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

    public SwerveDrive(PowerDistributionPanel pdp) {
        m_pdp = pdp;

        SmartDashboardTab.putData("SwerveDrive","swerveDriveSubsystem", this);
        if (RobotBase.isSimulation()) {

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
        if(RobotBase.isReal())
            return Rotation2d.fromDegrees(getHeading());
        else
            try {
                return swerveChassisSim.getHeading();
            } catch (Exception e) {
                return new Rotation2d();
            }
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

    public Pose2d getSimulatedChassisPose() {
        return swerveChassisSim.getPose();
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

        System.out.println("Input Rotation: " + rot);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        ); //from 2910's code
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        inputStates = swerveModuleStates;

//        // I'm not exactly sure why this needs to be inverted in teleop, but not auto?
//        if(RobotBase.isSimulation())
//            for(int i = 0; i < mSwerveModules.length; i++)
//                swerveModuleStates[i].angle = swerveModuleStates[i].angle.unaryMinus();


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
        inputRotationSpeed = kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond;
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        inputStates = desiredStates;
        mSwerveModules[0].setDesiredState(desiredStates[0]);
        mSwerveModules[1].setDesiredState(desiredStates[1]);
        mSwerveModules[2].setDesiredState(desiredStates[2]);
        mSwerveModules[3].setDesiredState(desiredStates[3]);
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
        // Update module positions based on the chassis' position, but keep the module heading
        for (int i = 0; i < mSwerveModules.length; i++) {
            var modulePositionFromChassis = modulePositions[i].rotateBy(getRotation()).plus(getPose().getTranslation());
            mSwerveModules[i].setPose(new Pose2d(modulePositionFromChassis, mSwerveModules[i].getHeading()));
        }

//        m_odometry.addVisionMeasurement(SimulationReferencePose.getRobotFieldPose(), Timer.getFPGATimestamp() + 0.2);

        System.out.println("Swerve Pose: " + getPose());
//        System.out.println("Field Pose: " + SimulationReferencePose.getRobotFieldPose());
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        m_odometry.resetPosition(pose, rotation);
        swerveChassisSim.setPose(pose);

        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setPose(pose);
            mSwerveModules[i].resetEncoders();
        }
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
//            System.out.println("Target Heading: " + getHeadingTarget());
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

    @Override
    public void simulationPeriodic() {
        double simChassisInputVoltage = (inputRotationSpeed / kMaxAngularSpeed) * RobotController.getBatteryVoltage() / 2 * inputRotationFudge;


        var testStates = inputStates;
        SwerveModuleState.optimize(testStates[0], new Rotation2d(mSwerveModules[0].getTurningRadians()));

        System.out.println("Raw Input States: " + inputStates[0]);
        System.out.println("Optimized States: " + testStates[0]);


        // If the input angle flips, you must rotate the chassis the other way
        double currentInputSign = Math.signum(inputStates[0].angle.getRadians());
        if(lastInputSign != currentInputSign){
            inputTurnInversion = -inputTurnInversion;
        }
        lastInputSign = currentInputSign;
        simChassisInputVoltage *= inputTurnInversion;

        swerveChassisSim.setInputs(simChassisInputVoltage, -simChassisInputVoltage);
        swerveChassisSim.update(0.02);

        System.out.println("Sim Position: " + swerveChassisSim.getPose());

        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
        angle.set(Math.IEEEremainder(-swerveChassisSim.getHeading().getDegrees(), 360));
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
