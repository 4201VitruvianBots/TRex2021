/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
Subsystem for interacting with the Limelight and OpenSight vision systems
 */

public class Vision extends SubsystemBase {
	// Variables for calculating distance
	private final double TARGET_HEIGHT = 98.25; // Outer port height above carpet in inches
	private final double LIMELIGHT_MOUNT_ANGLE = 32; // Angle that the Limelight is mounted at
	private final double LIMELIGHT_HEIGHT = 37.31; // Limelight height above the ground in inches

	private final double MIN_TARGET_DISTANCE = 1;
	private final double INNER_PORT_SLOPE = 1;
	private final double INNER_PORT_OFFSET = 1;

	private final double HORIZONTAL_TARGET_PIXEL_WIDTH = 1;
	private final double HORIZONTAL_TARGET_PIXEL_THRESHOLD = 1;
	private final double VERTICAL_TARGET_PIXEL_WIDTH = 1;
	private final double VERTICAL_TARGET_PIXEL_THRESHOLD = 1;

	// NetworkTables for reading vision data
	private NetworkTable limelight;
	private NetworkTable openSight;

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

		// Driver cam setup
//		camera = CameraServer.getInstance().startAutomaticCapture();
		camera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/video0");
	    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
	    camera.setExposureManual(25);
	    camera.setResolution(320, 240);
	    camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);

		//CameraServer.getInstance().addAxisCamera("opensight", "opensight.local");

	    // TODO: What port does opensight use?
		PortForwarder.add(6000, "opensight.local", 22);
		PortForwarder.add(5800, "10.42.1.11", 5800);
		PortForwarder.add(5801, "10.42.1.11", 5801);
		PortForwarder.add(5805, "10.42.1.11", 5805);

		// Init vision NetworkTables
		limelight = NetworkTableInstance.getDefault().getTable("limelight");
		openSight = NetworkTableInstance.getDefault().getTable("OpenSight");
		setPipeline(0);

		//initShuffleboard();
	}

	private void updateValidTarget() {
		// Determine whether the limelight has detected a valid target and not a random reflection
		// If the target is seen for a specific amount of time it is marked as valid
		if (hasTarget()) {
			setLastValidTargetTime();
		}
		if ((Timer.getFPGATimestamp() - lastValidTargetTime) < 3) {
			ledsOn();
			validTarget = true;
		} else {
			ledsOff();
			validTarget = false;
		}
	}

	public boolean getValidTarget() {
		return validTarget;
	}

	public void setLastValidTargetTime() {
		lastValidTargetTime = Timer.getFPGATimestamp();
	}

	// Limelight interaction functions
	public double getTargetY() {
		return limelight.getEntry("ty").getDouble(0);
	}

	public double getTargetX() {
		return limelight.getEntry("tx").getDouble(0);
	}

	public double getFilteredTargetX() {
		return targetXFilter.calculate(getTargetX());
	}
	
	public double getSmartTargetX() {
		if(getTargetDistance() > MIN_TARGET_DISTANCE) {
			double xDistance = Units.metersToFeet(m_swerveDrive.getPose().getTranslation().getX());
			double yDistance = Math.abs(Units.metersToFeet(m_swerveDrive.getPose().getTranslation().getY()));

			double maxYDistance = INNER_PORT_SLOPE * xDistance + INNER_PORT_OFFSET;

			if (yDistance < maxYDistance) {
				xDistance += 29.25 / 12.0;
				return innerTargetXFilter.calculate(Math.signum(getFilteredTargetX()) * Units.radiansToDegrees(Math.atan(xDistance / yDistance)));
			}
		}//recalibrates the distance from target based on the current position of the bot??

		return getFilteredTargetX();
	}

	private void resetPoseByVision() {
		if(!resetPose) {
			if((Math.abs(getHorizontalSidelength() - HORIZONTAL_TARGET_PIXEL_WIDTH) < HORIZONTAL_TARGET_PIXEL_THRESHOLD) &&
			   (Math.abs(getVerticalSidelength() - VERTICAL_TARGET_PIXEL_WIDTH) < VERTICAL_TARGET_PIXEL_THRESHOLD)) {
				double targetRadians = Units.degreesToRadians(m_turret.getFieldRelativeAngle());
				double xDistance = Math.abs(Math.cos(targetRadians)) * getTargetDistance();
				double yDistance = -Math.signum(getFilteredTargetX()) * Math.abs(Math.sin(targetRadians)) * getTargetDistance();

				m_swerveDrive.resetOdometry(new Pose2d(xDistance, yDistance, new Rotation2d()),
						Rotation2d.fromDegrees(m_swerveDrive.getHeading()));

				resetPose = true;
			}
		} else if(resetPose && !hasTarget()) {
			resetPose = false;
		}//resets pose of bot based on the vision of the bot
	}

	// More Limelight interaction functions

	public boolean hasTarget() {
		return limelight.getEntry("tv").getDouble(0) == 1;
	}

	public double getTargetArea() {
		return limelight.getEntry("ta").getDouble(0);
	}

	public double getTargetSkew() {
		return limelight.getEntry("ts").getDouble(0);
	}

	public double getPipelineLatency() {
		return limelight.getEntry("tl").getDouble(0);
	}

	public double getTargetShort() {
		return limelight.getEntry("tshort").getDouble(0);
	}

	public double getTargetLong() {
		return limelight.getEntry("tlong").getDouble(0);
	}

	public double getHorizontalSidelength() {
		return limelight.getEntry("thor").getDouble(0);
	}

	public double getVerticalSidelength() {
		return limelight.getEntry("tvert").getDouble(0);
	}

	public double getPipeline() {
		return limelight.getEntry("getpipe").getDouble(0);
	}

	public void ledsOn() {
		limelight.getEntry("ledMode").setNumber(3);
	}

	public void ledsOff() {
		limelight.getEntry("ledMode").setNumber(1);
	}

	public void setPipeline(int pipeline) {
		limelight.getEntry("pipeline").setNumber(pipeline);
	}

	// Calculate target distance based on field dimensions and the angle from the Limelight to the target
	public double getTargetDistance() {
		double angleToTarget = getPipeline() > 0 ? getTargetY() - 12.83 : getTargetY();

		double inches = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + angleToTarget));
		distances[index++ % distances.length] = inches / 12.0;

		return computeMode(distances);
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
    public double getPowerCellX() {
        // TODO: Calculate degrees from pixels?
        // return openSight.getEntry("found-x").getDouble(0) * 5.839; // 5.839 pixels per degree
        return openSight.getEntry("found-x").getDouble(0);
    }

	public boolean hasPowerCell() {
		return openSight.getEntry("found").getBoolean(false);
	}

	private void initShuffleboard() {
		// Unstable. Don''t use until WPILib fixes this
		Shuffleboard.getTab("Turret").addBoolean("Vision Valid Output", this::getValidTarget);
		Shuffleboard.getTab("Turret").addNumber("Vision Target X", this::getFilteredTargetX);

	}

	// set smartdashboard
	public void updateSmartDashboard() {
		SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
		SmartDashboard.putNumber("Limelight Target X", getTargetX());
		SmartDashboard.putNumber("Limelight Target Distance", getTargetDistance());
		SmartDashboard.putNumber("Limelight Pipeline", getPipeline());

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
