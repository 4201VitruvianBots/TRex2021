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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
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
  private TalonFX climbMotors[] = {new TalonFX(Constants.climbMotorA), new TalonFX(Constants.climbMotorB)};
  


  DoubleSolenoid climbPistons[] = {new DoubleSolenoid(Constants.pcmOne, Constants.climbPistonAForward, Constants.climbPistonAReverse), new DoubleSolenoid(Constants.pcmOne, Constants.climbPistonBForward, Constants.climbPistonBReverse)};

  private boolean climbState;

  public Climber() {
    // Set up climber motor
    climbMotors[0].configFactoryDefault();
    climbMotors[0].setSelectedSensorPosition(0);
    climbMotors[0].setNeutralMode(NeutralMode.Brake);
    climbMotors[0].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbMotors[1].configFactoryDefault();
    climbMotors[1].setSelectedSensorPosition(0);
    climbMotors[1].setNeutralMode(NeutralMode.Brake);
    climbMotors[1].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climbMotors[1].set(ControlMode.Follower, climbMotors[0].getDeviceID());

  }
  


  public boolean getClimbPistonExtendStatus(){
    return climbPistons[0].get() == DoubleSolenoid.Value.kForward ? true : false; //Gives the ClimbPistonExtendStatus if the climbPiston value equals the value of forward speed??

  }

  public void setClimbPistons(boolean state){
climbPistons[0].set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse); //sets values based on the ClimbPiston State
climbPistons[1].set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse); //sets values based on the ClimbPiston State
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
    climbMotors[0].set(ControlMode.PercentOutput, value);
  }
  
  public int getClimberPosition() {
	  return climbMotors[0].getSelectedSensorPosition();
  }

  //sets and configures the ClimberPosition
  public void setClimberPosition(double position) { 
    double setpoint = inchesToEncoderUnits(position);
    climbMotors[0].set(ControlMode.Position, setpoint);
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

    SmartDashboardTab.putNumber("Climber", "Position", encoderUnitsToInches(climbMotors[0].getSelectedSensorPosition()));
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