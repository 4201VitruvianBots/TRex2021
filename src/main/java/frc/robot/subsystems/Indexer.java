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
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for interacting with the robot's indexer (feeds balls from intake to shooter)
 */

public class Indexer extends SubsystemBase {
    private final double kI_Zone = 1;
    private final double maxVel = 1.1e4;
    private final double maxAccel = 1e6;
    private final double gearRatio = 1.0 / 27.0;

    // Setup indexer motor controller (SparkMax)
    CANSparkMax master = new CANSparkMax(Constants.indexerMotor, MotorType.kBrushless);
    CANEncoder encoder = master.getEncoder();
    CANPIDController pidController = master.getPIDController();

    // PID terms/other constants
    private double kF = 0.0001;
    private double kP = 0.000001;
    private double kI = 80;
    private double kD = 0.0001;

    public Indexer() {
        // Motor and PID controller setup
        master.restoreFactoryDefaults();
        master.setInverted(false);

        master.setIdleMode(IdleMode.kBrake);

        pidController.setFF(kF);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
        pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(1, 0);
        pidController.setIZone(kI_Zone);

        initShuffleboard();
    }

    // Self-explanatory commands

    public void setIndexerOutput(double output) {
        master.set(output);
    }

    public double getRPM() {
        return encoder.getVelocity() * gearRatio;
    }

    public void setRPM(double rpm) {
        double setpoint = rpm / gearRatio;
        SmartDashboard.putNumber("Indexer Setpoint", setpoint);
        pidController.setReference(setpoint, ControlType.kSmartVelocity);
    }

    private void initShuffleboard() {
        Shuffleboard.getTab("Indexer").addNumber("Carousel RPM", this::getRPM);
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putNumber("Indexer", "Carousel RPM", this.getRPM());
        SmartDashboard.putNumber("kF", kF);
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
    }

//    private void updatePIDValues() {
//        Allow PID values to be set through SmartDashboard ???
//        kF = SmartDashboard.getNumber("kF", 0);
//        kP = SmartDashboard.getNumber("kP", 0);
//        kI = SmartDashboard.getNumber("kI", 0);
//        kD = SmartDashboard.getNumber("kD", 0);
//        pidController.setFF(kF);
//        pidController.setP(kP);
//        pidController.setI(kI);
//        pidController.setD(kD);
//    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
        //updatePIDValues();
    }
}
