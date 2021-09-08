// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  private final Servo[] ShooterHoodServos = { new Servo(Constants.LeftShooterHoodServo),
      new Servo(Constants.RightShooterHoodServo) };
  private double SmartDashboardValue;

  public ShooterHood() {
    initShuffleboard();
  }

  public void setHoodAngle(double degrees) {
    if (degrees >= Constants.minHoodValue && degrees <= Constants.maxHoodValue) {
      ShooterHoodServos[0].setAngle(degrees);
      ShooterHoodServos[1].setAngle(Constants.maxHoodValue - degrees);
    }
    if (degrees < Constants.minHoodValue) {
      ShooterHoodServos[0].setAngle(Constants.minHoodValue);
      ShooterHoodServos[1].setAngle(Constants.maxHoodValue);
    }
    if (degrees > Constants.maxHoodValue) {
      ShooterHoodServos[0].setAngle(Constants.maxHoodValue);
      ShooterHoodServos[1].setAngle(Constants.minHoodValue);
    }
  }

  public void setHoodAngleToSmartDashboardValue() {
    ShooterHoodServos[0].setAngle(SmartDashboardValue);
    ShooterHoodServos[1].setAngle(Constants.maxHoodValue - SmartDashboardValue);
  }

  public double getHoodAngle() {
    return ShooterHoodServos[0].getAngle();
  }

  private void initShuffleboard() {
    SmartDashboardTab.putNumber("ShooterHood", "HoodAngle", getHoodAngle());
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("HoodAngle", getHoodAngle());
  }

  public void updateShooterAngles() {
    SmartDashboardValue = SmartDashboardTab.getNumber("ShooterHood", "HoodAngle", -1);
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    updateShooterAngles();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
