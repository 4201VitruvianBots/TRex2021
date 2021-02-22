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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
Subsystem for controlling the turret
 */

public class Turret extends SubsystemBase {
    private final int encoderUnitsPerRotation = 4096;
    private final SwerveDrive m_swerveDrive;
    // setup motor and encoder variables
    CANSparkMax turretMotor = new CANSparkMax(Constants.indexerMotor, MotorType.kBrushless);
    CANEncoder encoder = turretMotor.getEncoder();
    CANPIDController pidController = turretMotor.getPIDController();
    // Turret PID gains
    double kF = 0.07;     //0.05
    double kP = 0.2;    //0.155
    double kI = 0.0015;    //0.00075
    double kD = 0.0;  //0.00766
    int kI_Zone = 900;    //900 // 254: 1/kP?
    int kMaxIAccum = 1000000;//kI_Zone *3; //500000;    //900
    int kErrorBand = 50;//degreesToEncoderUnits(0.5);
    int kCruiseVelocity = 14000;
    int kMotionAcceleration = kCruiseVelocity * 10;
    // setup variables
    double[][] restrictedMovement = {{270, 300}, {60, 90}};
    double minAngle = -90;  // -135;
    double maxAngle = 90;   // 195;
    double gearRatio = 18.0 / 120.0;
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;

    public Turret(SwerveDrive swerveDrive) {
        this.m_swerveDrive = swerveDrive;
        // Setup turret motors
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(false);
        turretMotor.setIdleMode(IdleMode.kBrake);

        // Setup PID Controller
        pidController.setFF(kF);
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        double maxVel = 1.1e4;
        pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
        double maxAccel = 1e6;
        pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(1, 0);
        pidController.setIZone(kI_Zone);
    }

    // self-explanatory commands
    /**
    *this function is used to sync the actual turret direction and the encoder.
    */
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public int getControlMode() {
        return controlMode;
    }

    public void setControlMode(int mode) {
        controlMode = mode;
    }
    /**
    *this function gets the angle at wich the turret is facing reletave to the front of the robot (it's starting possition)
    */
    public double getTurretAngle() {
        return encoderUnitsToDegrees(encoder.getPosition());
    }

    /**
    *this function gets the angle of the robot relitive to the field.
    */
    public double getFieldRelativeAngle() {
        return getTurretAngle() - m_swerveDrive.getRawGyroAngle();
    }

    /**
    *this function determines the maximum angle that the turret can rotate relative to the drivetrain
    */
    public double getMaxAngle() {
        return maxAngle;
    }

    /**
    *this function determines the minimum angle that the turret can rotate relative to the drivetrain
    */
    public double getMinAngle() {
        return minAngle;
    }

    
    public boolean getTurretHome() {
//        return !turretHomeSensor.get(); cannot understand what the turret home sensor is refering to.
        return true;
    }

    public boolean getInitialHome() { //Checks if the bot is in its starting position??
        return initialHome;
    }

    /**
    *this function determines how fast the turret should rotate
    *@return max rpm of turret motor
    */
    public double getSetpoint() {
        return setpoint;
    }

    /**
    *this function finds the difference of the encoder reletave to the actual angle of the turret
    *@return the difference between the angle of the turret based off of the encoder vs. its actual angle.
    */
    public double getError() {
        return 0 - getTurretAngle();
    }

    /**
    *this function prevents the turret from rotationg past a certain degree reletave to the drivetrain (main purpose is to prevent turret wires from tangling and getting damaged)
    *@return i[0] < i[1] if not restricted, i[0] > i[1] if restricted.
    */
    public boolean isRestricted() {
        boolean value = false;
        for (double[] i : restrictedMovement) {
            if (i[0] < i[1]) {
                value = (this.getTurretAngle() > i[0] && this.getTurretAngle() < i[1]);
            } else {
                value = (this.getTurretAngle() > i[1] && this.getTurretAngle() < i[0]);
            }
        }
        return value;
    }

    /**
    *this function declares at what percent the turret motor should run at
    *@param output determines the precentage of power given to the motor
    */
    public void setPercentOutput(double output) {
        turretMotor.set(output);
    }

    // ???
    public void setRobotCentricSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // ???
    public void setFieldCentricSetpoint(double setpoint) {
        setpoint -= m_swerveDrive.getRawGyroAngle();

        if (setpoint > getMaxAngle())
            setpoint -= 360;
        else if (setpoint < getMinAngle())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    /**
    *this function declares the speed at wich the turret can rotate at
    */
    public void setClosedLoopPosition() {
        turretMotor.set(degreesToEncoderUnits(getSetpoint()));
    }

//    public void setSetpointOutput(double setpoint) {
//        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(setpoint));
//    }

    /**
    *this function uses the encoder to determine where the turret is facing
    *@return number of encoder units the turret has rotated
    */
    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));
    }

    /**
    *this function uses the encoder to determine where the turret is facing
    *@return number of Degrees the turret has rotated
    */
    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);
    }

    /**
    *this function determines if the turret is ready to fire at the powerport.
    *@return true if correctly aligned to shoot, false if not aligned.
    */
    public boolean onTarget() {
        return Math.abs(pidController.getSmartMotionAllowedClosedLoopError(1)) < kErrorBand; //not sure if this is the correct slot
    }

    // ???
//    public void clearIAccum() {
//        turretMotor.setIntegralAccumulator(0);
//    }

    private boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    // ???
    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output Current", turretMotor::getOutputCurrent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", this::getError);
//        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor::getIntegralAccumulator); can't find CANSparkmax equivalent
        Shuffleboard.getTab("Turret").addBoolean("Home", this::getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngle());
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output Current", turretMotor::getOutputCurrent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this::getTurretAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this::getFieldRelativeAngle);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this::getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", this::getError);
//        SmartDashboardTab.putNumber("Turret", "Turret Controller Setpoint", turretMotor.getClosedLoopTarget());  cannot understand what the getClosedLoopTarget is refering to. and can't find CANSparkmax equivalent
        SmartDashboardTab.putString("Turret", "Turret Control Mode", String.valueOf(this.getControlMode()));
//        SmartDashboardTab.putNumber("Turret", "Turret IAccum", turretMotor.getIntegralAccumulator()); can't find CANSparkmax equivalent
        SmartDashboardTab.putBoolean("Turret", "Home", getTurretHome());

        try {
            SmartDashboardTab.putString("DriveTrain", "Turret Command", this.getCurrentCommand().getName());
        } catch (Exception e) {

        }
        SmartDashboardTab.putNumber("Turret", "Control Mode", getControlMode());
    }

    @Override
    public void periodic() {
        if (getControlMode() == 1)
            setClosedLoopPosition();

        // This method will be called once per scheduler run
        // TODO: FIX
        if (!getTurretLatch() && getTurretHome()) {
//            turretMotor.setSelectedSensorPosition(0); no sensor on the CANSparkmax
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if (getTurretLatch() && !getTurretHome())
            setTurretLatch(false);

        if (!getInitialHome())
            if (getTurretHome())
                initialHome = true;

        updateSmartdashboard();
    }
}
