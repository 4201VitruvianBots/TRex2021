// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carousel extends SubsystemBase {
  private CANSparkMax carouselMotor = new CANSparkMax(Constants.carouselMotor, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public Carousel() {
    carouselMotor.restoreFactoryDefaults();
    carouselMotor.setInverted(false);
  }

  public void setPercentOutput(double output) {
    carouselMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
