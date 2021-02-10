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
Susbsystem for interacting with the robot's indexer (feeds balls from intake to shooter)
 */

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // Setup indexer motor controller (SparkMax)
  CANSparkMax master = new CANSparkMax(Constants.indexerMotor, MotorType.kBrushless);
  CANEncoder encoder = master.getEncoder();
  CANPIDController pidController = master.getPIDController();

  private double targetSetpoint;

  // PID terms/other constants
//  private double kF = 1.67;
//  private double kP = 2.36;
//  private double kI = 0;
//  private double kD = 1070;
  private double kF = 0.0001;
  private double kP = 0.000001;
  private double kI = 80;
  private double kD = 0.0001;

  private double kI_Zone = 1;
  private double maxVel = 1.1e4;
  private double maxAccel = 1e6;

  private double gearRatio = 1.0 / 27.0;

  private int controlMode = 1;

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

//    SmartDashboard.putNumber("kF", kF);
//    SmartDashboard.putNumber("kP", kP);
//    SmartDashboard.putNumber("kI", kI);
//    SmartDashboard.putNumber("kD", kD);
    //initShuffleboard();
  }

  // Self-explanatory commands

  public void toggleControlMode() {
    if(controlMode == 0)
      controlMode = 1;
    else
      controlMode = 0;
  }

  public int getControlMode() {
    return controlMode;
  }

  public void setIndexerOutput(double output) {
    master.set(output);
  }

  public void setRPM(double rpm) {
    double setpoint = rpm / gearRatio;
    SmartDashboard.putNumber("Indexer Setpoint", setpoint);
    pidController.setReference(setpoint, ControlType.kSmartVelocity);
  }

 public double getRPM() {
   return encoder.getVelocity() * gearRatio;
 }

  private void initShuffleboard() {
    Shuffleboard.getTab("Indexer").addNumber("Carousel RPM", this.getRPM());
  }

  private void updateSmartDashboard(){
    SmartDashboardTab.putBoolean("Indexer","Carousel RPM", this.getRPM());
  }

  private void updatePIDValues() {
    // Allow PID values to be set through SmartDashboard ???
    kF = SmartDashboard.getNumber("kF", 0);
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    pidController.setFF(kF);
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    //updatePIDValues();
  }
}
