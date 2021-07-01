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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants;

/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    private final int encoderUnitsPerRotation = 4096;
    private final SwerveDrive m_swerveDrive;
    // setup motor and encoder variables
    CANSparkMax turretMotor = new CANSparkMax(Constants.indexerMotor, MotorType.kBrushless);
    CANEncoder encoder = turretMotor.getEncoder();
    CANPIDController pidController = turretMotor.getPIDController();
    // Turret PID gains
    double kF = 0.07;     //0.05
    double kP = 0.2;    //0.155
    double kI = 0.0015;    //0.00075
    double kD = 0.0;  //0.00766
    int kI_Zone = 900;    //900 // 254: 1/kP?
    int kMaxIAccum = 1000000;//kI_Zone *3; //500000;    //900
    int kErrorBand = 50;//degreesToEncoderUnits(0.5);
    int kCruiseVelocity = 14000;
    int kMotionAcceleration = kCruiseVelocity * 10;
    // setup variables
    double[][] restrictedMovement = {{270, 300}, {60, 90}};
    double minAngle = -90;  // -135;
    double maxAngle = 90;   // 195;
    double gearRatio = 18.0 / 120.0;
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;

    public Turret(SwerveDrive swerveDrive) {
        this.m_swerveDrive = swerveDrive;
        // Setup turret motors
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(false);
        turretMotor.setIdleMode(IdleMode.kBrake);

        // Setup PID Controller
        pidController.setFF(kF);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        double maxVel = 1.1e4;
        pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
        double maxAccel = 1e6;
        pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(1, 0);
        pidController.setIZone(kI_Zone);
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

    public double getFieldRelativeAngle() {
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
        return 0 - getTurretAngle();
    }

    public boolean isRestricted() {
        boolean value = false;
        for (double[] i : restrictedMovement) {
            if (i[0] < i[1]) {
                value = (this.getTurretAngle() > i[0] && this.getTurretAngle() < i[1]);
            } else {
                value = (this.getTurretAngle() > i[1] && this.getTurretAngle() < i[0]);
            }
        }
        return value;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(output);
    }

    // ???
    public void setRobotCentricSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // ???
    public void setFieldCentricSetpoint(double setpoint) {
        setpoint -= m_swerveDrive.getHeadingDegrees();

        if (setpoint > getMaxAngle())
            setpoint -= 360;
        else if (setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPosition() {
        turretMotor.set(degreesToEncoderUnits(getSetpoint()));
    }

//    public void setSetpointOutput(double setpoint) {
//        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(setpoint));
//    }

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

    // ???
//    public void clearIAccum() {
//        turretMotor.setIntegralAccumulator(0);
//    }

    private boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    // ???
    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output Current", turretMotor::getOutputCurrent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", this::getError);
//        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor::getIntegralAccumulator); can't find CANSparkmax equivalent
        Shuffleboard.getTab("Turret").addBoolean("Home", this::getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngle());
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output Current", turretMotor::getOutputCurrent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", this::getError);
//        SmartDashboardTab.putNumber("Turret", "Turret Controller Setpoint", turretMotor.getClosedLoopTarget());  cannot understand what the getClosedLoopTarget is refering to. and can't find CANSparkmax equivalent
        SmartDashboardTab.putString("Turret", "Turret Control Mode", String.valueOf(this.getControlMode()));
//        SmartDashboardTab.putNumber("Turret", "Turret IAccum", turretMotor.getIntegralAccumulator()); can't find CANSparkmax equivalent
        SmartDashboardTab.putBoolean("Turret", "Home", getTurretHome());

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

//        updateSmartdashboard();
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
