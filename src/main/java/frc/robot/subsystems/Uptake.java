// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Uptake extends SubsystemBase {

  private TalonFX uptakeMotor = new TalonFX(Constants.uptakeMotor);

  /** Creates a new ExampleSubsystem. */
  public Uptake() {
    uptakeMotor.configFactoryDefault();
    uptakeMotor.setInverted(false);
  }

  public void setPercentOutput(double output) {
    uptakeMotor.set(ControlMode.PercentOutput, output);
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
