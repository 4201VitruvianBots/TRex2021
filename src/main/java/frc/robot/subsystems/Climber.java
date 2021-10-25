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
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
This class is the subsystem for the robot's climber
 */

public class Climber extends SubsystemBase {

  // setup variables

  private double gearRatio = 1.0/18.0;
  public double pulleyDiameter = 2.0; // inches

  // Climber motors and solenoid
  private TalonFX climbMotor = new TalonFX(Constants.climbMotor);

  DoubleSolenoid climbRatchet = new DoubleSolenoid(Constants.pcmOne, Constants.climbRatchetForward, Constants.climbRatchetReverse);
  DoubleSolenoid climbPistons = new DoubleSolenoid(Constants.pcmOne, Constants.climbPistonForward, Constants.climbPistonReverse);

  private boolean climbState;

  public Climber() {
    // Set up climber motor
    climbMotor.configFactoryDefault();
    climbMotor.setSelectedSensorPosition(0);
    climbMotor.setNeutralMode(NeutralMode.Brake);
    climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }
  


  public boolean getClimbPistonExtendStatus(){
    return climbPistons.get() == DoubleSolenoid.Value.kForward ? true : false; //Gives the ClimbPistonExtendStatus if the climbPiston value equals the value of forward speed??

  }
  public void setClimbRatchet(boolean state){
    climbRatchet.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse); //sets values based on the ClimbPiston State
  }

  public void setClimbPistons(boolean state){
    climbPistons.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse); //sets values based on the ClimbPiston State
  }

  public boolean getClimbState() {
    return climbState;
  }

  //getting climbstate based on the actual climbstate
  public void setClimbState(boolean climbState) {
    this.climbState = climbState;
  }

  public void setClimberOutput(double value) {
    // Prevent backdrive
    climbMotor.set(ControlMode.PercentOutput, value);
  }
  
  public double getClimberPosition() {
	  return climbMotor.getSelectedSensorPosition();
  }

  //sets and configures the ClimberPosition
  public void setClimberPosition(double position) { 
    double setpoint = inchesToEncoderUnits(position);
    climbMotor.set(ControlMode.Position, setpoint);
  }

  private double inchesToEncoderUnits(double inches) {
    return inches * gearRatio * (2048.0 / (Math.PI * pulleyDiameter)); //Returns the distance moved in encoder units from inches based on dimensions and gear ratio of the climber
  }

  private double encoderUnitsToInches(double encoderUnits) {
    return encoderUnits * (1/gearRatio) * ((Math.PI * pulleyDiameter) / 2048.0);
  }//Returns the distance moved in inches from encoder units based on dimensions and gear ratio of the climber??

  // Setting SmartDashboard
  private void updateShuffleboard(){
    SmartDashboard.putBoolean("Climb Mode", getClimbState());

    SmartDashboardTab.putNumber("Climber", "Position", encoderUnitsToInches(climbMotor.getSelectedSensorPosition()));
    SmartDashboardTab.putBoolean("Climber", "Climb Mode", climbState);
    SmartDashboardTab.putBoolean("Climber", "Climb Pistons", getClimbPistonExtendStatus());
//    try {
//      SmartDashboardTab.putString("Climber", "Current Command", this.getCurrentCommand().getName());
//    }  catch(Exception e) {
//
//    }
  }
  @Override
  public void periodic() {
    //updateShuffleboard();
  }
}