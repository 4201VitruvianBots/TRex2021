/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for controlling to robot's shooter
 */

public class Shooter extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     *
     * @return
     */

    // PID loop constants
    private final double kF = 0.618;  // 0.054      //  Gree: 0.0475;
    private final double kP = 7.25e-9;      //  0.4       //  0.00047
    private final double kI = 0.0;                    //  0.0000287
    private final double kD = 0.0;

    private final double kS = 0.618;
    private final double kV = 0.0708;
    private final double kA = 0.0239;
    // shooter motors
    private final TalonFX[] shooterMotors = {
            new TalonFX(Constants.flywheelMotorA),
            new TalonFX(Constants.flywheelMotorB),
    };
    private final Vision m_vision;
    public int kI_Zone = 100;
    public int kAllowableError = 50;
    // constants
    public double rpmOutput;
    public double rpmTolerance = 50.0;
    private double setpoint;
    private boolean timerStart;
    private double timeStamp;
    private boolean canShoot;
    public double gearRatio = 1.5;

    public Shooter(Vision vision, PowerDistributionPanel pdp) {
        // setup shooterMotors
        for (TalonFX outtakeMotor : shooterMotors) {
            outtakeMotor.configFactoryDefault();
            outtakeMotor.setNeutralMode(NeutralMode.Coast);
            outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            outtakeMotor.configVoltageCompSaturation(10);
            outtakeMotor.enableVoltageCompensation(true);
        }
        shooterMotors[0].setInverted(true);
        shooterMotors[1].follow(shooterMotors[0], FollowerType.PercentOutput);

        shooterMotors[0].config_kF(0, kF);
        shooterMotors[0].config_kP(0, kP);
        shooterMotors[0].config_kI(0, kI);
        shooterMotors[0].config_IntegralZone(0, kI_Zone);
        shooterMotors[0].config_kD(0, kD);
        shooterMotors[0].configAllowableClosedloopError(0, kAllowableError);
        shooterMotors[0].configClosedloopRamp(0.2);
        shooterMotors[1].configClosedloopRamp(0);
        shooterMotors[1].configOpenloopRamp(0);

        m_vision = vision;
        // parameters

        initShuffleboard();
    }

    //Self-explanatory commands

    public double getMotorInputCurrent(int motorIndex) {
        return shooterMotors[motorIndex].getSupplyCurrent();
    }

    public void setPower(double output) {
        shooterMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setRPM(double setpoint) {
        this.setpoint = RPMtoFalconUnits(setpoint);
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean canShoot() {
        return canShoot;
    }

    private void updateRPMSetpoint() {
        if (setpoint >= 0)
            shooterMotors[0].set(ControlMode.Velocity, setpoint);
        else
            setPower(0);
    }

    public void setTestRPM() {
        shooterMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }

    public boolean encoderAtSetpoint(int motorIndex) {
        return (Math.abs(shooterMotors[motorIndex].getClosedLoopError()) < 100.0);
    }

    /**
     *Returns the RMP of the shooter motor.
     *@param motorIndex The index of the measured motor
     *@return the rmp of the selected motor
     */
    public double getRPM(int motorIndex) {
        return falconUnitsToRPM(shooterMotors[motorIndex].getSelectedSensorVelocity());
    }

    public double getRotations(int motorIndex) {
        return (shooterMotors[motorIndex].getSelectedSensorPosition() / 2048.0) * gearRatio;
    }
    public double falconUnitsToRPM(double sensorUnits) {
        return (sensorUnits / 2048.0) * 60.0 * gearRatio;
    }

    public double RPMtoFalconUnits(double RPM) {
        return (RPM / 600.0) * 2048.0 / gearRatio;
    }

    // Smart Dashboard settings

    private void initShuffleboard() {
        SmartDashboard.putNumber("Flywheel Setpoint", setpoint);
        SmartDashboard.putNumber("Flywheel Encoder Units Setpoint", setpoint);
//        SmartDashboardTab.putNumber("Shooter", "RPM Output", rpmOutput);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kF", kF);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kP", kP);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kI", kI);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kD", kD);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kI_Zone", kI_Zone);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel kAllowableError", kAllowableError);
//        SmartDashboardTab.putNumber("Shooter", "Flywheel RPM Tolerance", rpmTolerance);
    }

    private void updateShuffleboard() {
        SmartDashboard.putNumber("Flywheel RPM", getRPM(0));
        SmartDashboard.putNumber("Motor Velocity", shooterMotors[0].getSelectedSensorVelocity());
        setRPM(SmartDashboard.getNumber("Flywheel Setpoint", 0));
//        setpoint = SmartDashboard.getNumber("Flywheel Encoder Units Setpoint", 0);

//        SmartDashboardTab.putNumber("Shooter", "RPM Primary", getRPM(0));
//        SmartDashboardTab.putNumber("Shooter", "RPM Secondary", getRPM(1));
//        SmartDashboardTab.putNumber("Shooter", "Setpoint", setpoint);
//        SmartDashboardTab.putNumber("Shooter", "Power", shooterMotors[0].getMotorOutputPercent());
//        SmartDashboardTab.putNumber("Shooter", "Error", getSetpoint() - getRPM(0));
//
//        SmartDashboardTab.putBoolean("DriveTrain", "CanShoot", canShoot());
    }

    public void updatePIDValues() {
        // Allow PID values to be set through SmartDashboard
        rpmOutput = SmartDashboardTab.getNumber("Shooter", "RPM Output", 0);
        rpmTolerance = SmartDashboardTab.getNumber("Shooter", "Flywheel RPM Tolerance", 0);

        shooterMotors[0].config_kF(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kF", 0));
        shooterMotors[0].config_kP(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kP", 0));
        shooterMotors[0].config_kI(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kI", 0));
        shooterMotors[0].config_IntegralZone(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kI_Zone", 0));
        shooterMotors[0].config_kD(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kD", 0));
        shooterMotors[0].configAllowableClosedloopError(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kAllowableError", 0));
    }


    @Override
    public void periodic() {
        updateRPMSetpoint();
//        updatePidRPM();
        updateShuffleboard();
//        updatePIDValues();

        if ((Math.abs(getSetpoint() - getRPM(0)) < getRPMTolerance()) && m_vision.hasTarget() &&
                (Math.abs(m_vision.getTargetX()) < 1) && !timerStart) {
            timerStart = true;
            timeStamp = Timer.getFPGATimestamp();
        } else if (((Math.abs(getSetpoint() - getRPM(0)) > getRPMTolerance()) || !m_vision.hasTarget() ||
                (Math.abs(m_vision.getTargetX()) > 1)) && timerStart) {
            timeStamp = 0;
            timerStart = false;
        }

        canShoot = timeStamp != 0 && Math.abs(Timer.getFPGATimestamp() - timeStamp) > 0.6;
    }
}
