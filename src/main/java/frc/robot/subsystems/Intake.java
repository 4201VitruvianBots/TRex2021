package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for interacting with the robot's intake
 */
/**
*this function determines the values of the PID loop for the intake motor.
*/
public class Intake extends SubsystemBase {
    // PID and FeedForward loop terms
    private double kFF = 0.00068; //0.06; //0.122
    private double kP = 6e-5; //0.492
    private double kI = 0;
    private double kD = 0;

    private double kI_Zone = 0;
    private double allowableError = 50;
    private double maxVel = 5880;
    private double maxAccel = 58800;
    private double gearRatio = 1.0 / 3.0;
    private boolean intaking = false;

    // Intake motor setup 
    private CANSparkMax intakeMotor =  new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
//  private CANEncoder intakeEncoder = intakeMotor.getEncoder();
//  private CANPIDController canPidController = intakeMotor.getPIDController();

    DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

    
    public Intake() {
        // Configure motors

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setInverted(false);

//    canPidController.setFF(kFF);
//    canPidController.setP(kP);
//    canPidController.setI(kI);
//    canPidController.setIZone(kI_Zone);
//    canPidController.setD(kD);
//    canPidController.setSmartMotionMaxVelocity(maxVel, 0);
//    canPidController.setSmartMotionMaxAccel(maxAccel, 0);
//    canPidController.setSmartMotionAllowedClosedLoopError(allowableError, 0);
    }

    // Self-explanatory functions

    /**
    *this function returns if the intake is active or inactive
    *@return true if intaking or false if inactive
    */
    public boolean getIntakingState() {
        return intaking;
    }

    /**
    *this function declares if the intake is active or inactive 
    *@param state if the intake is active or inactive
    */
    public void setIntakingState(boolean state) {
        intaking = state;
    }

    /**
    *this function returns if the double solenoid is active or inactive
    *@return true if extended or false if retracted
    */
    public boolean getIntakePistonExtendStatus(){
        return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    /**
    *this function extends and retracts the intake pistons
    *@param state if the intake is active or inactive
    */
    public void setintakePiston(boolean state){
        intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
    *this function determines the precentage of power used by the intake motor. Can be set from -1 to 1.
    *@param value the precentage of power that the motor is using
    */
    public void setIntakePercentOutput(double value){
        intakeMotor.set(value);
    }

//  public double getRPM(){
//    return intakeEncoder.getVelocity() * gearRatio;
//  }
//
//  public void setDirectRPM(double rpm){
//    canPidController.setReference(rpm, ControlType.kSmartVelocity);
//  }
//
//  public void setRPM(double rpm){
//    double setpoint =  rpm / gearRatio;
//    canPidController.setReference(setpoint, ControlType.kSmartVelocity);
//  }

    /**
    *this function displays the state of the intake onto the dashboard
    */
    private void updateSmartDashboard() {
        SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
        SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
    }
    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
