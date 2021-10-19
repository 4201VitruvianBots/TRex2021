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

    private final double gearRatio = 1.0 / 18.0;
    // Climber motors and solenoid
    private final TalonFX climbMotor = new TalonFX(Constants.climbMotor);
    public double pulleyDiameter = 2.0; // inches
    DoubleSolenoid ratchetPiston = new DoubleSolenoid(Constants.pcmOne, Constants.climbRatchetPistonForward, Constants.climbRatchetPistonReverse);
    DoubleSolenoid leftClimbPiston = new DoubleSolenoid(Constants.pcmOne, Constants.leftClimbPistonForward, Constants.leftClimbPistonReverse);
    DoubleSolenoid rightClimbPiston = new DoubleSolenoid(Constants.pcmOne, Constants.rightClimbPistonForward, Constants.rightClimbPistonReverse);

    private boolean climbState;

    public Climber() {
        // Set up climber motor
        climbMotor.configFactoryDefault();
        climbMotor.setSelectedSensorPosition(0);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    public boolean getClimbPistonExtendStatus() {
        return true;//climbPiston.get() == DoubleSolenoid.Value.kForward;
    }

    public void setRatchetPiston(boolean state) {
        ratchetPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void setLeftPiston(boolean state) {
        leftClimbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void setRightPiston(boolean state) {
        rightClimbPiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public boolean getClimbState() {
        return climbState;
    }

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

    public void setClimberPosition(double position) {
        double setpoint = inchesToEncoderUnits(position);
        climbMotor.set(ControlMode.Position, setpoint);
    }

    private double inchesToEncoderUnits(double inches) {
        return inches * gearRatio * (2048.0 / (Math.PI * pulleyDiameter));
    }

    private double encoderUnitsToInches(double encoderUnits) {
        return encoderUnits * (1 / gearRatio) * ((Math.PI * pulleyDiameter) / 2048.0);
    }

    private void updateShuffleboard() {
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