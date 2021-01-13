/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */

    // setup variables

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

    double minAngle = -90;  // -135;
    double maxAngle = 90;   // 195;
    double gearRatio = 18.0 / 120.0;
    private double setpoint = 0; //angle

    private int encoderUnitsPerRotation = 4096;
    private int controlMode = 1;
    private boolean initialHome;

    private final SwerveDrive m_swerveDrive;

    private Timer timeout = new Timer();

    private CANCoder encoder = new CANCoder(Constants.turretEncoder);

    private VictorSPX turretMotor = new VictorSPX(Constants.turretMotor);

    private DigitalInput turretHomeSensor = new DigitalInput(Constants.turretHomeSensor);
    private boolean turretHomeSensorLatch = false;

    public Turret(SwerveDrive swerveDrive) {
        // Setup turrent motors
        m_swerveDrive = swerveDrive;
        encoder.configFactoryDefault();
        encoder.setPositionToAbsolute();
        encoder.configSensorDirection(true);

        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        turretMotor.config_kF(0, kF);
        turretMotor.config_kP(0, kP);
        turretMotor.config_kI(0, kI);
        turretMotor.config_IntegralZone(0, kI_Zone);
        turretMotor.configMaxIntegralAccumulator(0, kMaxIAccum);
        turretMotor.config_kD(0, kD);
        turretMotor.configMotionCruiseVelocity(kCruiseVelocity);
        turretMotor.configMotionAcceleration(kMotionAcceleration);
        turretMotor.configAllowableClosedloopError(0, kErrorBand);

        //turretPID.enableContinuousInput(0, 360);

        //initShuffleboard();
    }

    // self-explanatory comnmands

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
        encoder.setPosition(0);
    }

    public void setControlMode(int mode) {
        controlMode = mode;
    }

    public int getControlMode() {
        return controlMode;
    }

    public double getTurretAngle() {
        return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
    }

    public double getFieldRelativeAngle() {
        return getTurretAngle() - m_swerveDrive.getRawGyroAngle();
    }

    public double getMaxAngle() {
        return maxAngle;
    }

    public double getMinAngle() {
        return minAngle;
    }

    public boolean getTurretHome() {
        return !turretHomeSensor.get();
    }

    public boolean getInitialHome() { //Checks if the bot is in its starting position??
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    // ???
    public void setRobotCentricSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // ???
    public void setFieldCentricSetpoint(double setpoint) {
        setpoint -= m_swerveDrive.getRawGyroAngle();

        if (setpoint > getMaxAngle())
            setpoint -= 360;
        else if (setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    // ???
    public void setClosedLoopPosition() {
        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(getSetpoint()));
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
        return Math.abs(turretMotor.getClosedLoopError()) < kErrorBand;
    }

    // ???
    public void clearIAccum() {
        turretMotor.setIntegralAccumulator(0);
    }

    // ???
    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    private boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output", turretMotor::getMotorOutputPercent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", turretMotor::getClosedLoopError);
        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor::getIntegralAccumulator);
        Shuffleboard.getTab("Turret").addBoolean("Home", this::getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngle());

        SmartDashboardTab.putNumber("Turret", "Turret Motor Output", turretMotor.getMotorOutputPercent());
        SmartDashboardTab.putNumber("Turret", "Turret Robot Relative Angle", getTurretAngle());
        SmartDashboardTab.putNumber("Turret", "Turret Field Relative Angle", getFieldRelativeAngle());
        SmartDashboardTab.putNumber("Turret", "Turret Setpoint", getSetpoint());
//    SmartDashboardTab.putNumber("Turret", "Turret Error", turretMotor.getClosedLoopError());
//    SmartDashboardTab.putNumber("Turret", "Turret Controller Setpoint", turretMotor.getClosedLoopTarget());
//    SmartDashboardTab.putString("Turret", "Turret Control Mode", turretMotor.getControlMode().toString());
//    SmartDashboardTab.putNumber("Turret", "Turret IAccum", turretMotor.getIntegralAccumulator());
        SmartDashboardTab.putBoolean("Turret", "Home", getTurretHome());

//        try {
//            SmartDashboardTab.putString("DriveTrain", "Turret Command", this.getCurrentCommand().getName());
//        }catch (Exception e) {
//
//        }
//    SmartDashboardTab.putNumber("Turret", "Control Mode", getControlMode());
    }

    @Override
    public void periodic() {
        if (getControlMode() == 1)
            setClosedLoopPosition();

        // This method will be called once per scheduler run
        // TODO: FIX
        if (!getTurretLatch() && getTurretHome()) {
            turretMotor.setSelectedSensorPosition(0);
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if (getTurretLatch() && !getTurretHome())
            setTurretLatch(false);

        if (!initialHome)
            if (getTurretHome())
                initialHome = true;

        updateSmartdashboard();
    }
}
