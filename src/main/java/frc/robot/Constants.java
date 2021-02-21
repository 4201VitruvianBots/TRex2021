/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static final int frontRightDriveMotor = 0;
    public static final int frontRightTurningMotor = 1;
    public static final int frontLeftDriveMotor = 2;
    public static final int frontLeftTurningMotor = 3;
    public static final int backLeftDriveMotor = 4;
    public static final int backLeftTurningMotor = 5;
    public static final int backRightDriveMotor = 6;
    public static final int backRightTurningMotor = 7;

    public static final double freeMotorSpeedRPS = 638.0 / 6.0;

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(30);
        //Distance between centers of right and left wheels on robot. Meters?
        public static final double kWheelBase = Units.inchesToMeters(30);
        //Distance between front and back wheels on robot. Meters?
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Tool suite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14);

        public static final double kvVoltSecondsPerRadian = 3.41; // originally 1.5
        public static final double kaVoltSecondsSquaredPerRadian = 0.111; // originally 0.3

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
    }

    public static final class ModuleConstants {

        public static final double kDriveMotorGearRatio = 6.89;
        public static final int kTurningMotorGearRatio = 12;

        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI / (kTurningMotorGearRatio * freeMotorSpeedRPS);
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR*kDriveMotorGearRatio);

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ((double) kEncoderCPR* kTurningMotorGearRatio);

        public static final double kPModuleTurningController = 1;

        public static final double kPModuleDriveController = 1.57;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
        public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 12;
        public static final double kPYController = 12;
        public static final double kPThetaController = 1;

        public static final double kIXController = 0;
        public static final double kIYController = 0;
        public static final double kDXController = 0;
        public static final double kDYController = 0;

        //Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);

    }
    public static int intakeMotor = 47;
    public static final int pcmOne = 11;
    public static final int intakePistonForward = 2; // 2
    public static final int intakePistonReverse = 3; // 3
    public static enum IntakeStates {
        INTAKE_EMPTY, INTAKE_ONE_BALL, INTAKE_FOUR_BALLS, INTAKE_FIVE_BALLS
    }
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;
    public static final int turretEncoder = 61;
    public static final int turretMotor = 60;
    public static final int turretHomeSensor = 30;
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;
    public static final int climbPistonAForward = 31;
    public static final int climbPistonAReverse = 32;
    public static final int climbPistonBForward = 33;
    public static final int climbPistonBReverse = 34;
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;

    // DIO

    public static final int xEncoderPortA = 0;
    public static final int xEncoderPortB = 1;
    public static final int yEncoderPortA = 2;
    public static final int yEncoderPortB = 3;
    public static final int rotationEncoderPortA = 4;
    public static final int rotationEncoderPortB = 5;

    // placeholder values
    public static final int uptakeMotor = 4201;

    // Shooting physics constants
    public static final double g = 9.81; // Absolute value, in meters per second squared
    public static final double verticalTargetDistance = Units.inchesToMeters(98.25 - 38); // Distance between shooter and target heights from ground
    public static final double verticalShooterAngle = Math.PI / 3; // Angle ball is shot from shooter relative to the ground 
    // TODO: Find accurate measurements for T-rex

}
