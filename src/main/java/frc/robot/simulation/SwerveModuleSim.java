package frc.robot.simulation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N7;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveModuleSim {

    private SwerveDriveKinematics m_kinematics;

    private SwerveModule[] m_swerveModules;
    private DifferentialDrivetrainSim[] m_simulatedModules;

    private double turnFudge = 0;

    public SwerveModuleSim(SwerveDriveKinematics kinematics, SwerveModule[] swerveModules){
        m_kinematics = kinematics;
        m_swerveModules = swerveModules;

        // TODO: Make this more general
        m_simulatedModules = new DifferentialDrivetrainSim[m_swerveModules.length];
        for(int i =0; i < m_swerveModules.length; i++) {
            m_simulatedModules[i] = new DifferentialDrivetrainSim(
                    DriveConstants.kDrivetrainPlant,
                    DCMotor.getFalcon500(1),
                    Constants.ModuleConstants.kDriveMotorGearRatio,
                    Units.inchesToMeters(1.5), // What is this value?
                    Constants.ModuleConstants.kWheelDiameterMeters / 2.0,
                    null
            );
        }
    }

    public void update(double dt) {
        for(int i = 0; i < m_swerveModules.length; i++) {
            double leftInput = m_swerveModules[i].getDriveOutput() + (m_swerveModules[i].getTurnOutput() * turnFudge);
            double rightInput = m_swerveModules[i].getDriveOutput() - (m_swerveModules[i].getTurnOutput() * turnFudge);


            // Normalize the wheel speeds
            double maxMagnitude = Math.max(Math.abs(leftInput), Math.abs(rightInput));
            if (maxMagnitude > 1.0) {
                leftInput /= maxMagnitude;
                rightInput /= maxMagnitude;
            }

            double leftVolts = leftInput * RobotController.getBatteryVoltage();
            double rightVolts = rightInput * RobotController.getBatteryVoltage();

            m_simulatedModules[i].setInputs(leftVolts, rightVolts);
            m_simulatedModules[i].update(dt);

            double moduleVelocity = (m_simulatedModules[i].getLeftVelocityMetersPerSecond() + m_simulatedModules[i].getRightVelocityMetersPerSecond())/ 2;

            var updatedModuleState = new SwerveModuleState(moduleVelocity, m_simulatedModules[i].getHeading());
            System.out.println("H:" + m_simulatedModules[i].getHeading());
            m_swerveModules[i].setSimulatedState(updatedModuleState);
        }
        System.out.println();
    }
}
