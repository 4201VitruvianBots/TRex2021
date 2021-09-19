/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.net.PortForwarder;

/*
Subsystem for interacting with our vision system
 */

public class Vision extends SubsystemBase {
	// Variables for calculating distance
	private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
	private final double CAMERA_MOUNT_ANGLE = 32; // Angle that the camera is mounted at
	private final double CAMERA_HEIGHT = 37.31; // Camera height above the ground in inches

	private final double MIN_TARGET_DISTANCE = 1;
	private final double INNER_PORT_SLOPE = 1;
	private final double INNER_PORT_OFFSET = 1;

	private final double HORIZONTAL_TARGET_PIXEL_WIDTH = 1;
	private final double HORIZONTAL_TARGET_PIXEL_THRESHOLD = 1;
	private final double VERTICAL_TARGET_PIXEL_WIDTH = 1;
	private final double VERTICAL_TARGET_PIXEL_THRESHOLD = 1;

	// NetworkTables for reading vision data
	private NetworkTable oak_1;
	private NetworkTable oak_2;
	private NetworkTable oak_d_l;
	private NetworkTable oak_d_i;

	// Subsystems that will be controlled based on vision data
    private final SwerveDrive m_swerveDrive;
	private final Turret m_turret;

	private boolean resetPose;

	private double lastValidTargetTime;
	private boolean validTarget;

	double[] distances = new double[5];
	double[] counts = new double[5];
	int index = 0;

	// Filters to prevent target values from oscillating too much
	SlewRateLimiter targetXFilter = new SlewRateLimiter(20);
	SlewRateLimiter innerTargetXFilter = new SlewRateLimiter(20);

	UsbCamera camera;

	public Vision(SwerveDrive swerveDrive, Turret turret) {
		
		m_swerveDrive = swerveDrive;
		m_turret = turret;

		if (RobotBase.isReal() && false) {
			// Driver cam setup
//		camera = CameraServer.getInstance().startAutomaticCapture();
//		camera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/video0");
//	    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
//	    camera.setExposureManual(25);
//	    camera.setResolution(320, 240);
//	    camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
		}
		//CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");

	    // TODO: What port does opensight use?
//		PortForwarder.add(6000, "opensight.local", 22);
//		PortForwarder.add(5800, "10.42.1.11", 5800);
//		PortForwarder.add(5801, "10.42.1.11", 5801);
//		PortForwarder.add(5805, "10.42.1.11", 5805);

		// Init vision NetworkTables
		oak_1 = NetworkTableInstance.getDefault().getTable("OAK-1");

		//initShuffleboard();
	}

	private void updateValidTarget() {
		// Determine whether the camera has detected a valid target and not random noise
		// If the target is seen for a specific amount of time it is marked as valid
		if (hasTarget()) {
			setLastValidTargetTime();
		}
		if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 2) {
			validTarget = true;
		} else {
			validTarget = false;
		}
	}

	public boolean hasTarget() {
		return oak_1.getEntry("tv").getDouble(0) == 1;
	}

	public double getGoalX() {
		return oak_1.getEntry("tx").getDouble(0);
	}

	public String getGoalLabel() {
		return oak_1.getEntry("target_label").getString("");
	}

	public double getFilteredTargetX() {
		return targetXFilter.calculate(getGoalX());
	}

	public boolean getValidTarget() {
		return validTarget;
	}

	public void setLastValidTargetTime() {
		lastValidTargetTime = Timer.getFPGATimestamp();
	}


	// Calculate target distance based on field dimensions and the angle from the camera to the target
	public double getTargetDistance() {
		//TODO: Update
//		double angleToTarget = getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();
//
//		double inches = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE + angleToTarget));
//		distances[index++ % distances.length] = inches / 12.0;
//
//		return computeMode(distances);
		return 0;
	}

	// Used to find the most common value to provide accurate target data
	private double computeMode(double[] data) {
		// Compute mode
		this.counts = new double[data.length];
		for (int i = 0; i < data.length; i++) {
			for (int j = 0; j < data.length; j++) {
				if (data[i] == data[j]) {
					this.counts[i]++;
				}
			}
		}

		int highestIndex = 0;
		double previousHigh = 0;
		for (int i = 0; i < this.counts.length; i++) {
			if (this.counts[i] > previousHigh) {
				highestIndex = i;
				previousHigh = this.counts[i];
			}
		}

		return data[highestIndex]; // Final distance in feet
	}

	// Read ball position data from OpenSight (Raspberry Pi)
//    public double getPowerCellX() {
//        // TODO: Calculate degrees from pixels?
//        // return openSight.getEntry("found-x").getDouble(0) * 5.839; // 5.839 pixels per degree
//        return openSight.getEntry("found-x").getDouble(0);
//    }

//	public boolean hasPowerCell() {
//		return openSight.getEntry("found").getBoolean(false);
//	}

	private void initShuffleboard() {
		// Unstable. Don''t use until WPILib fixes this
		Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this::getValidTarget);
		Shuffleboard.getTab("Turret").addNumber("Vision Target X", this::getFilteredTargetX);
	}

	// set smartdashboard
	public void updateSmartDashboard() {
		SmartDashboard.putBoolean("OAK-1 Has Target", hasTarget());
		SmartDashboard.putNumber("OAK-1 Target X", getGoalX());
		SmartDashboard.putString("OAK-1 Target Label", getGoalLabel());

		SmartDashboardTab.putBoolean("Turret", "Vision Valid Output", getValidTarget());
		SmartDashboardTab.putNumber("Turret", "Vision Target X", getFilteredTargetX());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		updateSmartDashboard();
		updateValidTarget();

		//resetPoseByVision();
	}
}
