// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/** Represents a simulated swerve turn module. This basically copies the flywheel sim class,
 * but adds distance and angle measurements. */
public class SwerveModuleTurnSim extends LinearSystemSim<N1, N1, N1> {
    // Gearbox for the flywheel.
    private final DCMotor m_gearbox;

    // The gearing from the motors to the output.
    private final double m_gearing;
    private final double m_moduleRadius;

    private double m_angleRadians;

    /**
     * Creates a simulated flywheel mechanism.
     *
     * @param plant The linear system that represents the flywheel.
     * @param gearbox The type of and number of motors in the flywheel gearbox.
     * @param gearing The gearing of the flywheel (numbers greater than 1 represent reductions).
     */
    public SwerveModuleTurnSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox, double gearing, double moduleRadius) {
        super(plant);
        m_gearbox = gearbox;
        m_gearing = gearing;
        m_moduleRadius = moduleRadius;
    }

    /**
     * Creates a simulated flywheel mechanism.
     *
     * @param plant The linear system that represents the flywheel.
     * @param gearbox The type of and number of motors in the flywheel gearbox.
     * @param gearing The gearing of the flywheel (numbers greater than 1 represent reductions).
     * @param measurementStdDevs The standard deviations of the measurements.
     */
    public SwerveModuleTurnSim(
            LinearSystem<N1, N1, N1> plant,
            DCMotor gearbox,
            double gearing,
            double moduleRadius,
            Matrix<N1, N1> measurementStdDevs) {
        super(plant, measurementStdDevs);
        m_gearbox = gearbox;
        m_gearing = gearing;
        m_moduleRadius = moduleRadius;
    }

//    /**
//     * Creates a simulated flywheel mechanism.
//     *
//     * @param gearbox The type of and number of motors in the flywheel gearbox.
//     * @param gearing The gearing of the flywheel (numbers greater than 1 represent reductions).
//     * @param jKgMetersSquared The moment of inertia of the flywheel. If this is unknown, use the
//     *     {@link #FlywheelSim(LinearSystem, DCMotor, double, Matrix)} constructor.
//     */
//    @SuppressWarnings("ParameterName")
//    public SwerveModuleTurnSim(DCMotor gearbox, double gearing, double jKgMetersSquared, double moduleRadius) {
//        super(LinearSystemId.identifyPositionSystem(gearbox, jKgMetersSquared, gearing));
//        m_gearbox = gearbox;
//        m_gearing = gearing;
//        m_moduleRadius = moduleRadius;
//    }
//
//    /**
//     * Creates a simulated flywheel mechanism.
//     *
//     * @param gearbox The type of and number of motors in the flywheel gearbox.
//     * @param gearing The gearing of the flywheel (numbers greater than 1 represent reductions).
//     * @param jKgMetersSquared The moment of inertia of the flywheel. If this is unknown, use the
//     *     {@link #FlywheelSim(LinearSystem, DCMotor, double, Matrix)} constructor.
//     * @param measurementStdDevs The standard deviations of the measurements.
//     */
//    @SuppressWarnings("ParameterName")
//    public SwerveModuleTurnSim(
//            DCMotor gearbox, double gearing, double moduleRadius, double jKgMetersSquared, Matrix<N1, N1> measurementStdDevs) {
//        super(
//                LinearSystemId.identifyPositionSystem(gearbox, jKgMetersSquared, gearing),
//                measurementStdDevs);
//        m_gearbox = gearbox;
//        m_gearing = gearing;
//        m_moduleRadius = moduleRadius;
//    }

    /**
     * Returns the flywheel velocity.
     *
     * @return The flywheel velocity.
     */
    public double getAngularVelocityRadPerSec() {
        return getOutput(0);
    }

    /**
     * Returns the flywheel velocity in RPM.
     *
     * @return The flywheel velocity in RPM.
     */
    public double getAngularVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(getOutput(0));
    }

    /**
     * Returns the flywheel velocity.
     *
     * @return The flywheel velocity.
     */
    public double getArcLength() {
        return m_angleRadians * m_moduleRadius;
    }

    /**
     * Returns the flywheel velocity.
     *
     * @return The flywheel velocity.
     */
    public double getAngleRadians() {
        return m_angleRadians;
    }

    /**
     * Returns the flywheel velocity.
     *
     * @return The flywheel velocity.
     */
    public double getAngle() {
        return Units.radiansToDegrees(getAngleRadians());
    }

    /**
     * Returns the flywheel velocity.
     *
     * @return The flywheel velocity.
     */
    public void resetRadians() {
        m_angleRadians = 0;
    }
    /**
     * Returns the flywheel current draw.
     *
     * @return The flywheel current draw.
     */
    @Override
    public double getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
        // 2x faster than the flywheel
        return m_gearbox.getCurrent(getAngularVelocityRadPerSec() * m_gearing, m_u.get(0, 0))
                * Math.signum(m_u.get(0, 0));
    }

    /**
     * Sets the input voltage for the flywheel.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts) {
        setInput(volts);
    }

    @Override
    public void update(double dtSeconds) {
        super.update(dtSeconds);
        m_angleRadians += getAngularVelocityRadPerSec() * dtSeconds;
    }
}
