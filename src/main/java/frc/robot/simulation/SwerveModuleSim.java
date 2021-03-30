package frc.robot.simulation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N7;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

import static frc.robot.Constants.DriveConstants.kTrackWidth;
import static frc.robot.Constants.DriveConstants.kWheelBase;

public class SwerveModuleSim {
    private Translation2d[] swervePositions = {
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    private SwerveDriveKinematics m_kinematics;

    private SwerveModule[] m_swerveModules;
    private DifferentialDrivetrainSim[] m_simulatedModules;
    private FlywheelSim m_turnTest;
    private FlywheelSim[] m_testModules;

    private double turnFudge = 0.125;
    private double outputFudge = 3.5;
    private double[] turnRadians = {0,0,0,0};

    public SwerveModuleSim(SwerveDriveKinematics kinematics, SwerveModule[] swerveModules){
        m_kinematics = kinematics;
        m_swerveModules = swerveModules;

        double ksVolts = 0.587;
        double kvVoltSecondsPerMeter = 2.3;
        double kaVoltSecondsSquaredPerMeter = 0.0917;

        double kMaxSpeedMetersPerSecond = Units.feetToMeters(14);

        double kvVoltSecondsPerRadian = 6.41; // originally 1.5
        double kaVoltSecondsSquaredPerRadian = 0.111; // originally 0.3

        LinearSystem<N2, N2, N2> kSwerveModulePlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

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
            m_simulatedModules[i].setPose(new Pose2d(swervePositions[i], new Rotation2d()));
        }
        var throttleModel = LinearSystemId.identifyVelocitySystem(DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter);

        m_testModules = new FlywheelSim[m_swerveModules.length];
        for(int i =0; i < m_testModules.length; i++) {
            m_testModules[i] = new FlywheelSim(
                    throttleModel,
                    DCMotor.getFalcon500(1),
                    Constants.ModuleConstants.kDriveMotorGearRatio,
                    null
            );
        }
    }

    public void update(double dt) {
        for(int i = 0; i < m_swerveModules.length; i++) {
            double leftInput = m_swerveModules[i].getDriveOutput() - (m_swerveModules[i].getTurnOutput() * turnFudge);
            double rightInput = m_swerveModules[i].getDriveOutput() + (m_swerveModules[i].getTurnOutput() * turnFudge);

            // Normalize the wheel speeds
            double maxMagnitude = Math.max(Math.abs(leftInput), Math.abs(rightInput));
            if (maxMagnitude > 1.0) {
                leftInput /= maxMagnitude;
                rightInput /= maxMagnitude;
            }

            double leftVolts = leftInput * RobotController.getBatteryVoltage() * outputFudge;
            double rightVolts = rightInput * RobotController.getBatteryVoltage() * outputFudge;

//            m_testModules[i].setInputVoltage(m_swerveModules[i].getDriveOutput()* RobotController.getBatteryVoltage());
//            m_testModules[i].update(dt);
//
//            double moduleVelocity = m_testModules[i].getAngularVelocityRPM() * Math.PI * Constants.ModuleConstants.kWheelModuleDiameter;
//            var updatedModuleState = new SwerveModuleState(moduleVelocity, new Rotation2d());
//            m_swerveModules[i].setSimulatedState(updatedModuleState);

            m_simulatedModules[i].setInputs(leftVolts, rightVolts);
            m_simulatedModules[i].update(dt);

            double moduleVelocity = (m_simulatedModules[i].getLeftVelocityMetersPerSecond() + m_simulatedModules[i].getRightVelocityMetersPerSecond())/ 2;

//            double turnInput = m_swerveModules[i].getTurnOutput();
//            m_turnTest.setInputVoltage(turnInput * RobotController.getBatteryVoltage());
//            m_turnTest.update(dt);
//            turnRadians[i] += (m_turnTest.getAngularVelocityRadPerSec() * dt);

//            var updatedModuleState = new SwerveModuleState(moduleVelocity, m_simulatedModules[i].getHeading());
            var updatedModuleState = new SwerveModuleState(moduleVelocity, m_simulatedModules[i].getHeading());
            m_swerveModules[i].setSimulatedState(updatedModuleState);
        }
    }

    public Pose2d[] getSimPoses(){
        return new Pose2d[] {
                m_simulatedModules[0].getPose(),
                m_simulatedModules[1].getPose(),
                m_simulatedModules[2].getPose(),
                m_simulatedModules[3].getPose()
        };
    }

    public void setPosesFromChassis(Pose2d robotPose){
        Rotation2d[] chassisRotation = {
            robotPose.getRotation(),
            robotPose.getRotation(),
            robotPose.getRotation(),
            robotPose.getRotation()
        };
        setPosesFromChassis(robotPose, chassisRotation);
    }

    public void setPosesFromChassis(Pose2d robotPose, Rotation2d[] moduleHeadings){
        for(int i = 0; i < m_simulatedModules.length; i++) {
            var translation = robotPose.getTranslation().plus(swervePositions[i]);
            m_simulatedModules[i].setPose(new Pose2d(translation, robotPose.getRotation()));
        }
    }

    public void stopSimulatedModules() {
        for(int i = 0; i < m_simulatedModules.length; i++) {
            m_simulatedModules[i].setPose(m_simulatedModules[i].getPose());
        }
    }

    public Rotation2d getChassisHeading() {
        double headingRadians = 0;
        for(int i = 0; i < m_simulatedModules.length; i++) {
            headingRadians += m_simulatedModules[i].getHeading().unaryMinus().getRadians();
        }
        return new Rotation2d(headingRadians / 4);
    }
}
