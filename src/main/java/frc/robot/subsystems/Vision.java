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
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
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
	private final NetworkTable goal_camera;
	private final NetworkTable intake_camera;
	private final NetworkTable indexer_camera;

	private int goal_camera_type = 0; // 2: PhotonVision

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

		// Init vision NetworkTables
		switch (goal_camera_type) {
			case 2:
				goal_camera = NetworkTableInstance.getDefault().getTable("photonvision");
				break;
			case 1:
				goal_camera = NetworkTableInstance.getDefault().getTable("limelight");
				break;
			case 0:
			default:
				goal_camera = NetworkTableInstance.getDefault().getTable("OAK-D_Goal");
				break;
		}
		intake_camera = NetworkTableInstance.getDefault().getTable("OAK-1_Intake");
		indexer_camera = NetworkTableInstance.getDefault().getTable("OAK-1_Indexer");

		PortForwarder.add(4200, "10.42.1.100", 80);
		PortForwarder.add(4201, "10.42.1.100", 5801);
		PortForwarder.add(4200, "10.42.1.101", 80);
		PortForwarder.add(4201, "10.42.1.101", 5801);;
		PortForwarder.add(4201, "10.42.1.101", 5802);

		//initShuffleboard();
	}

	private void updateValidTarget() {
		// Determine whether the camera has detected a valid target and not random noise
		// If the target is seen for a specific amount of time it is marked as valid
		if (hasGoalTarget()) {
			setLastValidTargetTime();
		}
		if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 2) {
			validTarget = true;
		} else {
			validTarget = false;
		}
	}

	public boolean hasGoalTarget() {
		switch (goal_camera_type) {
			case 2:
				return goal_camera.getEntry("hasTarget").getBoolean(false);
			case 1:
			case 0:
			default:
				return goal_camera.getEntry("tv").getDouble(0) == 1;
		}
	}

	public double getGoalX() {
		switch (goal_camera_type) {
			case 2:
				return goal_camera.getEntry("targetYaw").getDouble(0);
			case 1:
			case 0:
			default:
				return goal_camera.getEntry("tx").getDouble(0);
		}
	}

	public String getGoalLabel() {
		return goal_camera.getEntry("target_label").getString("");
	}

	public double getTargetDistance() {
		return goal_camera.getEntry("tz").getDouble(0);
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

	// Read ball position data from OAK-1_Intake
	public double getPowerCellX() {
		double[] nullValue = {-99};
		var values = intake_camera.getEntry("ta").getDoubleArray(nullValue);

		if(values[0] == -99) {
			return 0;
		} else {
			return values[0];
		}
	}

	public boolean hasPowerCell() {
		return intake_camera.getEntry("tv").getDouble(0) == 1;
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
		SmartDashboard.putBoolean("OAK-D_Goal Has Target", hasGoalTarget());
		SmartDashboard.putNumber("OAK-D_Goal Target X", getGoalX());
		SmartDashboard.putNumber("OAK-D_Goal Target Distance", getTargetDistance());

		SmartDashboardTab.putBoolean("Turret", "Vision Valid Output", getValidTarget());
		SmartDashboardTab.putNumber("Turret", "Vision Target X", getFilteredTargetX());

		SmartDashboardTab.putNumber("OAK", "Vision Target X", getFilteredTargetX());
		SmartDashboardTab.putNumber("OAK", "Vision Target X", getFilteredTargetX());
		SmartDashboardTab.putNumber("OAK", "Vision Target X", getFilteredTargetX());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		updateSmartDashboard();
		updateValidTarget();

		//resetPoseByVision();
	}
}
