/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants;

/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    // setup motor and encoder variables
    CANSparkMax turretMotor = new CANSparkMax(Constants.turretMotor, MotorType.kBrushless);
    CANEncoder encoder = turretMotor.getEncoder();
    CANPIDController pidController = turretMotor.getPIDController();
//    DigitalInput turretHomeSensor = new DigitalInput(Constants.turretHomeSensor);

    // Turret PID gains
    double kF = 0.006;
    double kP = 0.04;
    double kI = 0.0004;
    double kD = 0.2;
    int kI_Zone = 4;
    int kErrorBand = 1;//degreesToEncoderUnits(0.5);
    int kCruiseVelocity = 14000;
    int kMotionAcceleration = kCruiseVelocity * 10;

    // setup variables
    double minAngle = -45;
    double maxAngle = 45;
    private final int encoderUnitsPerRotation = 42;
    double gearRatio = 28.0 / 120.0;    // TODO: Ratio is correct, but values are wrong
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;

    public Turret(SwerveDrive swerveDrive) {
        this.m_swerveDrive = swerveDrive;
        // Setup turret motors
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(true);
        turretMotor.setIdleMode(IdleMode.kBrake);
        // turretMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.25);
        // turretMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)-0.25);

        // Setup PID Controller
        pidController.setFF(kF);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        double maxVel = 1.1e4;
        pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
        double maxAccel = 1e6;
        pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(kErrorBand, 0);
        pidController.setIZone(kI_Zone);

        resetEncoder();
        setAbsoluteSetpoint(0);

        // initShuffleboard();
    }

    // self-explanatory commands
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public int getControlMode() {
        return controlMode;
    }

    public void setControlMode(int mode) {
        controlMode = mode;
    }

    public double getTurretAngle() {
        return encoderUnitsToDegrees(encoder.getPosition());
    }

    public double getRobotRelativeAngle() {
        return getTurretAngle() - m_swerveDrive.getHeadingDegrees();
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public boolean getTurretHome() {
//        return !turretHomeSensor.get(); cannot understand what the turret home sensor is refering to.
        return true;
    }

    public boolean getInitialHome() { //Checks if the bot is in its starting position??
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getError() {
        return getSetpoint() - getTurretAngle();
    }

    public void setPercentOutput(double output) {
        turretMotor.set(output);
    }

    public void setAbsoluteSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setRobotCentricSetpoint(double setpoint) {
        setpoint -= m_swerveDrive.getHeadingDegrees();

        if (setpoint > getMaxAngle())
            setpoint -= 360;
        else if (setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPosition() {
        double angle = Math.max(Math.min(getSetpoint(), maxAngle),  minAngle);
        pidController.setReference(degreesToEncoderUnits(angle), ControlType.kSmartMotion);
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);
    }

    // checks if the turret is pointing within the tolerance of the target
    public boolean onTarget() {
        return Math.abs(pidController.getSmartMotionAllowedClosedLoopError(1)) < kErrorBand; //not sure if this is the correct slot
    }

    private boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    private void initShuffleboard() {
        SmartDashboardTab.putNumber("Turret", "Turret Setpoint", getSetpoint());
        SmartDashboardTab.putNumber("Turret", "Turret Absolute Relative Angle", getTurretAngle());
        SmartDashboardTab.putNumber("Turret", "Turret Error", getError());
        SmartDashboardTab.putNumber("Turret", "Turret kF", kF);
        SmartDashboardTab.putNumber("Turret", "Turret kP", kP);
        SmartDashboardTab.putNumber("Turret", "Turret kI", kI);
        SmartDashboardTab.putNumber("Turret", "Turret kD", kD);
        SmartDashboardTab.putNumber("Turret", "Turret kI_Zone", kI_Zone);
        SmartDashboardTab.putNumber("Turret", "Turret kAllowableError", kErrorBand);
//        Shuffleboard.getTab("Turret").addBoolean("Home", this::getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        SmartDashboard.putNumber("Turret Angle", getRobotRelativeAngle());
        SmartDashboardTab.putNumber("Turret", "Turret Motor Output Current", turretMotor.getOutputCurrent());
        SmartDashboardTab.putNumber("Turret","Turret Absolute Angle", getTurretAngle());
       SmartDashboardTab.putNumber("Turret","Turret Setpoint", getSetpoint());
        SmartDashboardTab.putNumber("Turret","Turret Error", getError());
//        SmartDashboardTab.putNumber("Turret", "Turret Controller Setpoint", turretMotor.getClosedLoopTarget());  cannot understand what the getClosedLoopTarget is refering to. and can't find CANSparkmax equivalent
        SmartDashboardTab.putString("Turret", "Turret Control Mode", String.valueOf(this.getControlMode()));
//        SmartDashboardTab.putNumber("Turret", "Turret IAccum", turretMotor.getIntegralAccumulator()); can't find CANSparkmax equivalent
        SmartDashboardTab.putBoolean("Turret", "Home", getTurretHome());

        SmartDashboardTab.putNumber("Turret","Turret Encoder Counts", encoder.getPosition());

        // setpoint = SmartDashboardTab.getNumber("Turret", "Turret Setpoint", getSetpoint());
        // kF = SmartDashboardTab.getNumber("Turret", "Turret kF", kF);
        // kP = SmartDashboardTab.getNumber("Turret", "Turret kP", kP);
        // kI = SmartDashboardTab.getNumber("Turret", "Turret kI", kI);
        // kD = SmartDashboardTab.getNumber("Turret", "Turret kD", kD);
        // kI_Zone = (int)SmartDashboardTab.getNumber("Turret", "Turret kI_Zone", kI_Zone);
        // kErrorBand = (int)SmartDashboardTab.getNumber("Turret", "Turret kAllowableError", kErrorBand);
        // pidController.setFF(kF);
        // pidController.setP(kP);
        // pidController.setI(kI);
        // pidController.setIZone(kI_Zone);
        // pidController.setD(kD);
        try {
            SmartDashboardTab.putString("DriveTrain", "Turret Command", this.getCurrentCommand().getName());
        } catch (Exception e) {

        }
        SmartDashboardTab.putNumber("Turret", "Control Mode", getControlMode());
    }

    @Override
    public void periodic() {
        if (getControlMode() == 1)
            setClosedLoopPosition();

        // This method will be called once per scheduler run
        // TODO: FIX
        if (!getTurretLatch() && getTurretHome()) {
//            turretMotor.setSelectedSensorPosition(0); no sensor on the CANSparkmax
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if (getTurretLatch() && !getTurretHome())
            setTurretLatch(false);

        if (!getInitialHome())
            if (getTurretHome())
                initialHome = true;

        updateSmartdashboard();
    }

    public double getTurretSimAngle(){
        return getTurretAngle() + 180;
    }

    public Pose2d getTurretSimPose() {
        return new Pose2d(m_swerveDrive.getPose().getX(),
                m_swerveDrive.getPose().getY(),
                new Rotation2d(Math.toRadians(getTurretSimAngle())));
    }

    @Override
    public void simulationPeriodic() {
    }

    public double getIdealTargetDistance() {
        return Math.sqrt(Math.pow(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), 2) + Math.pow(SimConstants.blueGoalPose.getX() - getTurretSimPose().getX(), 2));
    }

    public double getIdealTurretAngle() {

        double targetRadians = Math.atan2(SimConstants.blueGoalPose.getY() - getTurretSimPose().getY(), SimConstants.blueGoalPose.getX() - getTurretSimPose().getX());

        return Math.toDegrees(targetRadians);
    }
}
