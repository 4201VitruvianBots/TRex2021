/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    // setup variables

    // PID loop constants
    private double kF = 0.0523;  // 0.054      //  Gree: 0.0475;
    private double kP = 0.6;      //  0.4       //  0.00047
    private double kI = 0.0;                    //  0.0000287
    private double kD = 0.0;

//    private double kF = 0.0523;  // 0.054      //  Gree: 0.0475;
//    private double kP = 0.6;      //  0.4       //  0.00047
//    private double kI = 0.0;                    //  0.0000287
//    private double kD = 0.0;

    private double kS = 0.155;
    private double kV = 0.111;
    private double kA = 0.02;

//    private double kP = 1.91;
//    private double kI = 0.0;
//    private double kD = 0.0;

    public int kI_Zone = 100;
    public int kAllowableError = 50;

    private TalonFX[] outtakeMotors = {
            new TalonFX(Constants.flywheelMotorA),
            new TalonFX(Constants.flywheelMotorB),
    };

    public double rpmOutput;
    public double rpmTolerance = 50.0;

    private double setpoint;
    private boolean timerStart;
    private double timestamp;
    private boolean canShoot;

    private PowerDistributionPanel m_pdp;
    private Vision m_vision;

//    public PIDController flywheelController = new PIDController(kP, kI, kD);
//    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public Shooter(Vision vision, PowerDistributionPanel pdp) {
        // Setup shooter motors (Falcons)
        for (TalonFX outtakeMotor : outtakeMotors) {
            outtakeMotor.configFactoryDefault();
            outtakeMotor.setNeutralMode(NeutralMode.Coast);
            outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            outtakeMotor.configVoltageCompSaturation(10);
            outtakeMotor.enableVoltageCompensation(true);
        }
        outtakeMotors[0].setInverted(true);
        outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);

        outtakeMotors[0].config_kF(0, kF);
        outtakeMotors[0].config_kP(0, kP);
        outtakeMotors[0].config_kI(0, kI);
        outtakeMotors[0].config_IntegralZone(0, kI_Zone);
        outtakeMotors[0].config_kD(0, kD);
        outtakeMotors[0].configAllowableClosedloopError(0, kAllowableError);
        outtakeMotors[0].configClosedloopRamp(0.2);
        outtakeMotors[1].configClosedloopRamp(0);
        outtakeMotors[1].configOpenloopRamp(0);
        
        m_vision = vision;
        m_pdp = pdp;

        initShuffleboard();
    }

    //Self-explanatory commands

    public double getMotorInputCurrent(int motorIndex) {
        return outtakeMotors[motorIndex].getSupplyCurrent();
    }

    public void setPower(double output) {
        outtakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setRPM(double setpoint) {
        this.setpoint = setpoint;
    }

//    private void updatePidRPM() {
//        if (setpoint >= 0)
//            outtakeMotors[0].set(ControlMode.Velocity, setpoint, DemandType.ArbitraryFeedForward,
//                    feedforward.calculate(setpoint / 60.0));
//        else
//            setPower(0);
//    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean canShoot() {
        return canShoot;
    }

    private void updateRPMSetpoint() {
        if (setpoint >= 0)
            outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(setpoint));
        else
            setPower(0);
    }

    public void setTestRPM() {
        outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }

    public boolean encoderAtSetpoint(int motorIndex) {
        return (Math.abs(outtakeMotors[motorIndex].getClosedLoopError()) < 100.0);
    }

    public double getRPM(int motorIndex) {
        return falconUnitsToRPM(outtakeMotors[motorIndex].getSelectedSensorVelocity());
    }

    public double falconUnitsToRPM(double sensorUnits) {
        return (sensorUnits / 2048.0) * 600.0;
    }

    public double RPMtoFalconUnits(double RPM) {
        return (RPM / 600.0) * 2048.0;
    }

    // Smart Dashboard settings
    
    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
//    Shuffleboard.getTab("Shooter").addNumber("RPM Primary", () -> this.getRPM(0));
//    Shuffleboard.getTab("Shooter").addNumber("RPM Secondary", () -> this.getRPM(1));
//    Shuffleboard.getTab("Shooter").addNumber("Power", () -> this.outtakeMotors[0].getMotorOutputPercent());

        SmartDashboardTab.putNumber("Shooter", "RPM Output", rpmOutput);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kF", kF);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kP", kP);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kI", kI);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kD", kD);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kI_Zone", kI_Zone);
        SmartDashboardTab.putNumber("Shooter", "Flywheel kAllowableError", kAllowableError);
        SmartDashboardTab.putNumber("Shooter", "Flywheel RPM Tolerance", rpmTolerance);
    }

    private void updateShuffleboard() {
        SmartDashboard.putNumber("RPM", falconUnitsToRPM(outtakeMotors[0].getSelectedSensorVelocity()));

        SmartDashboardTab.putNumber("Shooter", "RPM Primary", getRPM(0));
        SmartDashboardTab.putNumber("Shooter", "RPM Secondary", getRPM(1));
        SmartDashboardTab.putNumber("Shooter", "Setpoint", setpoint);
        SmartDashboardTab.putNumber("Shooter", "Power", outtakeMotors[0].getMotorOutputPercent());
        SmartDashboardTab.putNumber("Shooter", "Error", getSetpoint()-getRPM(0));

        SmartDashboardTab.putBoolean("DriveTrain", "CanShoot", canShoot());
    }

    public void updatePIDValues() {
        // Allow PID values to be set through SmartDashboard
        rpmOutput = SmartDashboardTab.getNumber("Shooter", "RPM Output", 0);
        rpmTolerance = SmartDashboardTab.getNumber("Shooter", "Flywheel RPM Tolerance", 0);

        outtakeMotors[0].config_kF(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kF", 0));
        outtakeMotors[0].config_kP(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kP", 0));
        outtakeMotors[0].config_kI(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kI", 0));
        outtakeMotors[0].config_IntegralZone(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kI_Zone", 0));
        outtakeMotors[0].config_kD(0, SmartDashboardTab.getNumber("Shooter", "Flywheel kD", 0));
        outtakeMotors[0].configAllowableClosedloopError(0, (int) SmartDashboardTab.getNumber("Shooter", "Flywheel kAllowableError", 0));
    }


    @Override
    public void periodic() {
        updateRPMSetpoint();
//        updatePidRPM();
        updateShuffleboard();
        updatePIDValues();

        if ((Math.abs(getSetpoint() - getRPM(0)) < getRPMTolerance()) && m_vision.hasTarget() && 
        		(Math.abs(m_vision.getTargetX()) < 1) && !timerStart) {
            timerStart = true;
            timestamp = Timer.getFPGATimestamp();
        } else if (((Math.abs(getSetpoint() - getRPM(0)) > getRPMTolerance()) || !m_vision.hasTarget() || 
        		(Math.abs(m_vision.getTargetX()) > 1)) && timerStart) {
            timestamp = 0;
            timerStart = false;
        }

        if (timestamp != 0) {
            if (Math.abs(Timer.getFPGATimestamp() - timestamp) > 0.6)
                canShoot = true;
            else
                canShoot = false;
            
        } else
            canShoot = false;
    }
}
