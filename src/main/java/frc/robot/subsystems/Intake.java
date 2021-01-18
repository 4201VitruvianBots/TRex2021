package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for interacting with the robot's intake
 */

public class Intake extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    private boolean intakeState = false;

    // Intake motor setup
    private TalonFX intakeMotor =  new TalonFX(Constants.intakeMotor);
    DoubleSolenoid intakePistons = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

    public Intake() {
        // Configure motors
        intakeMotor.configFactoryDefault();
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
        intakeMotor.configOpenloopRamp(0.1);
        intakeMotor.configClosedloopRamp(0.1);
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.configForwardSoftLimitEnable(false);
        intakeMotor.configReverseSoftLimitEnable(false);
        intakeMotor.setInverted(false);
    }

    // Self-explanatory functions

    public boolean getIntakeState() {
        return intakeState;
    }

    public void setIntakeState(boolean state) {
        intakeState = state;
    }
    public boolean getIntakePistonExtendStatus(){
        return intakePistons.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    public void setIntakePiston(boolean state){
        intakePistons.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void setIntakePercentOutput(double value){
        intakeMotor.set(ControlMode.PercentOutput, value);
    }

    private void updateSmartDashboard() {
        SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakeState());
        SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
    }
    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
