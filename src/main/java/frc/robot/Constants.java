/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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
    public static final int frontLeftDriveMotor = 26;
    public static final int frontLeftTurningMotor = 27;
    public static final int frontRightDriveMotor = 24;
    public static final int frontRightTurningMotor = 25;
    public static final int backLeftDriveMotor = 22;
    public static final int backLeftTurningMotor = 23;
    public static final int backRightDriveMotor = 20;
    public static final int backRightTurningMotor = 21;

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.5;
        //Distance between centers of right and left wheels on robot. Meters?
        public static final double kWheelBase = 0.5;
        //Distance between front and back wheels on robot. Meters?

        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.587;
        public static final double kvVoltSecondsPerMeter = 2.3;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0917;

        public static final double kMaxSpeedMetersPerSecond = 3;
    }

    public static final class ModuleConstants {
        public static final double  kDriveMotorGearRatio = 8.16; //6.89 to 1
        public static final double kTurningMotorGearRatio = 12.8; //12 to 1
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.10; //10.16 cm
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4*2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4*2 * Math.PI;

        public static final double kDriveEncoderDistancePerPulse =
                (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR*kDriveMotorGearRatio);

        public static final double kTurningEncoderDistancePerPulse =
                (double) ((2.0 * Math.PI) / (kTurningMotorGearRatio * kEncoderCPR));

        public static final double kPModuleTurningController = 0.6;

        public static final double kPModuleDriveController = 0.25;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.25; //4.383024
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        //Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);

    }

    public static final int pcmOne = 11;
    public static final int intakePistonForward = 0; // 2
    public static final int intakePistonReverse = 1; // 3
    public static enum IntakeStates {
        INTAKE_EMPTY, INTAKE_ONE_BALL, INTAKE_FOUR_BALLS, INTAKE_FIVE_BALLS
    }
    public static final int flywheelMotorA = 40;
    public static int intakeMotor = 47;
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
    public static final int indexerMotor = 31;
    public static final int uptakeMotor = 35;

    // Undetermined IDs
    public static final int carouselMotor = 4201;
}
