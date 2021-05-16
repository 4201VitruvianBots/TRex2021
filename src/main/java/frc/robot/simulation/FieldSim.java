package frc.robot.simulation;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import static frc.robot.Constants.DriveConstants.kTrackWidth;
import static frc.robot.Constants.DriveConstants.kWheelBase;

public class FieldSim {
    private Field2d m_field2d;
    private final SwerveDrive m_swerveDrive;
    private final Powercell[] m_powercells = new Powercell[17];

    private int ballCount;

    private double m_autoStartTime;

    public FieldSim(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;

        for(int i = 0; i < m_powercells.length; i++)
            m_powercells[i] = new Powercell(String.format("PowerCell_" + String.format("%02d", i) ));

        m_field2d = new Field2d();
    }

    public Field2d getField2d() {
        return m_field2d;
    }

    public void initSim() {
        // Load 3 powercells into the robot
        for(int i = 0; i < 3; i++)
            m_powercells[i].setBallState(1);
        for(int i = 3; i < m_powercells.length; i++)
            m_powercells[i].setBallState(0);

        ballCount = 3;

        // Put 3 powercells in the trench;
        /*m_powercells[3].setBallPose(SimConstants.blueTrenchBallPos[0]);
        m_powercells[4].setBallPose(SimConstants.blueTrenchBallPos[1]);
        m_powercells[5].setBallPose(SimConstants.blueTrenchBallPos[2]);
        m_powercells[6].setBallPose(SimConstants.blueTrenchBallPos[3]);
        m_powercells[7].setBallPose(SimConstants.blueTrenchBallPos[4]);
        m_powercells[8].setBallPose(SimConstants.blueCenterBalls[3]);
        m_powercells[9].setBallPose(SimConstants.blueCenterBalls[4]);
        m_powercells[10].setBallPose(SimConstants.redTrenchBallPos[0]);
        m_powercells[11].setBallPose(SimConstants.redTrenchBallPos[1]);
        m_powercells[12].setBallPose(SimConstants.redTrenchBallPos[2]);
        m_powercells[13].setBallPose(SimConstants.redTrenchBallPos[3]);
        m_powercells[14].setBallPose(SimConstants.redTrenchBallPos[4]);
        m_powercells[15].setBallPose(SimConstants.redCenterBalls[3]);
        m_powercells[16].setBallPose(SimConstants.redCenterBalls[4]);*/

        Pose2d startPosition = new Pose2d(Units.inchesToMeters(30),Units.inchesToMeters(30), new Rotation2d(Units.degreesToRadians(30)));
        m_field2d.setRobotPose(startPosition);
        m_swerveDrive.resetOdometry(m_field2d.getRobotPose(), startPosition.getRotation());
//        m_autoStartTime = Timer.getFPGATimestamp();

        // Workaround for image loading bug
        m_field2d.getObject("trajectory").setPose(new Pose2d());

//        m_field2d.getObject("Heading Target").setPose(new Pose2d());
    }

    private void updateModulePoses() {
        /* Intake Points:
           ^: Front of the robot
              -------
             |   ^   |
             |       |
             0-------1
             |       |
             3-------2
         */

        // Look up rotating a point about another point in 2D space for the math explanation
//        Pose2d robotPose = m_swerveDrive.getPose();
//        Translation2d [] ModuleLocations = {
//            new Translation2d(kWheelBase/2, kTrackWidth/2),
//            new Translation2d(kWheelBase/2, -kTrackWidth/2),
//            new Translation2d(-kWheelBase/2, kTrackWidth/2),
//            new Translation2d(-kWheelBase/2, -kTrackWidth/2)
//        };
//        for (int i = 0; i < ModuleLocations.length; i++) {
//            Translation2d updatedPositions = ModuleLocations[i].rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
//            SwerveModulePose[i] = new Pose2d(updatedPositions,m_swerveDrive.getSwerveModule(i).getHeading().plus(m_swerveDrive.getRotation()));
//        }
    }

    private boolean isBallInIntakeZone(Pose2d ballPose){
        return false;

        // The rise/run between intake points 0 to 1
        // Since the intake is a rectangle, this is the same as the slope between points 2 to 3
        /*double slope0to1 = (intakePose[1].getY() - intakePose[0].getY()) /(intakePose[1].getX() - intakePose[0].getX());

        // The rise/run between points 1 to 2
        // Same as slope between points 3 and 0
        double slope1to2 = (intakePose[2].getY() - intakePose[1].getY()) /(intakePose[2].getX() - intakePose[1].getX());

        // Use point-slope form to check if ball pose is above or below each line on the intake rectangle
        // For each pair of parallel lines, the ball needs to be above one line and below the other
        // Note: it's very important that the points be in the same order as the diagram above
        return (
                (ballPose.getY() >= slope0to1 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY()) ==
                        (ballPose.getY() <= slope0to1 * (ballPose.getX() - intakePose[2].getX()) + intakePose[2].getY())
        ) && (
                (ballPose.getY() >= slope1to2 * (ballPose.getX() - intakePose[0].getX()) + intakePose[0].getY()) ==
                        (ballPose.getY() <= slope1to2 * (ballPose.getX() - intakePose[1].getX()) + intakePose[1].getY())
        );
*/
        /*List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();
        for (Pose2d p:intakePose) {
            xValues.add(p.getX());
            yValues.add(p.getY());
        }
        // This is technically cheating, need a more accurate comparison
        double minX = Collections.min(xValues, null);
        double maxX = Collections.max(xValues, null);
        double minY = Collections.min(yValues, null);
        double maxY = Collections.max(yValues, null);
        if (maxX > ballPose.getX() && ballPose.getX() > minX &&
            maxY > ballPose.getY() && ballPose.getY() > minY)
            return true;
        else
            return false;*/
    }

    /*  Sometimes, the auto paths ran will eject the robot out of bounds. This will reset the robot state so you can
        re-run the auto without restarting the sim
     */
    public void disabledInit() {
//        m_swerveDrive.resetOdometry(m_field2d.getRobotPose(), Arrays.stream(m_swerveDrive.getSimPoses()).map(Pose2d::getRotation).toArray(Rotation2d[]::new));
    }

    public void simulationPeriodic() {
//        var robotPose = m_field2d.getRobotPose();

//        if(robotPose.getX() < 0 || robotPose.getX() > SimConstants.fieldWidth ||
//                robotPose.getY() < 0 || robotPose.getY() > SimConstants.fieldHieght)
//            resetRobotPose(new Pose2d(SimConstants.fieldWidth / 2.0 ,SimConstants.fieldHieght / 2.0 , new Rotation2d(0)));

        m_field2d.setRobotPose(m_swerveDrive.getPose());

        // m_field2d.getObject("Turret").setPose(new Pose2d(m_swerveDrive.getPose().getTranslation(),
        //         new Rotation2d(Math.toRadians(getIdealTurretAngle()))));

        updateModulePoses();

        // m_field2d.getObject("Swerve Modules").setPoses(m_swerveDrive.getModulePoses());

        for(Powercell p:m_powercells) {
            updateBallState(p);
        }

//        m_field2d.getObject("Heading Target").setPose(m_swerveDrive.getTargetPose());

        m_field2d.getObject("PowerCells").setPoses(Arrays.stream(m_powercells).map(Powercell::getBallPose)
                                                                .collect(Collectors.toList()));
//        m_field2d.getObject("Heading Target").setPose(new Pose2d(m_swerveDrive.getTargetPose().getTranslation(), m_swerveDrive.getHeadingToTarget()));

        SmartDashboard.putData("Field2d", m_field2d);
    }

//     public double getAutoStartTime(){
//         return m_autoStartTime;
//     }
//     public double getIdealTargetDistance() {
//         return Math.sqrt(Math.pow(SimConstants.blueGoalPose.getY() - m_turret.getTurretSimPose().getY(), 2) + Math.pow(SimConstants.blueGoalPose.getX() - m_turret.getTurretSimPose().getX(), 2));
//     }

//     public double getIdealTurretAngle() {

// //        double targetRadians = Math.atan2(SimConstants.blueGoalPose.getY() -m_turret.getTurretSimPose().getY(), SimConstants.blueGoalPose.getX() - m_turret.getTurretSimPose().getX());
//         double targetRadians = Math.atan2(SimConstants.redGoalPose.getY() -m_turret.getTurretSimPose().getY(), SimConstants.redGoalPose.getX() - m_turret.getTurretSimPose().getX());
//         return Math.toDegrees(targetRadians);
//     }

    public Powercell[] getPowerCells() {
        return m_powercells;
    }

    public Pose2d getRobotPose() {
        return m_field2d.getRobotPose();
    }

    public synchronized void resetRobotPose(Pose2d pose){
        m_field2d.setRobotPose(pose);
        m_swerveDrive.resetOdometry(pose, pose.getRotation());
    }

    private void updateBallState(Powercell powercell) {
        Pose2d ballPose = powercell.getBallPose();
        if(powercell.getBallState() != 1) {
            if (ballPose.getX() < 0 || ballPose.getX() > SimConstants.fieldWidth || ballPose.getY() < 0 || ballPose.getY() > SimConstants.fieldHieght)
                powercell.setBallState(3);
        }

//        System.out.println("Ball Shot: " + wasShot + "\tBall State: " + ballState);
//        System.out.println("Ball State: " + ballState + "\tCos: " + ballPose.getRotation().getCos() + "\tX Pos: " + ballPose.getX());
        switch (powercell.getBallState()) {
            case 3:
                // Ball is out of bounds
                if(ballPose.getX() < SimConstants.fieldWidth / 2.0)
                    powercell.setBallPose(SimConstants.redLoadingStation);
                else
                    powercell.setBallPose(SimConstants.blueLoadingStation);

                powercell.setBallState(0);
                break;
            case 2:
                // Ball is traveling in the air
                double currentTime = RobotController.getFPGATime();
                // FPGA time is in microseonds, need to convert it into seconds
                double deltaT = (currentTime - powercell.getLastTimestamp()) / 1e6;
                double distanceTraveled = SimConstants.shotSpeed * deltaT;
                double deltaX = distanceTraveled * ballPose.getRotation().getCos();
                double deltaY = distanceTraveled * ballPose.getRotation().getSin();
//                System.out.println("Delta X: " + deltaX + "\tDelta Y: " + deltaY + "\tDelta T: " + deltaT);
                powercell.setBallPose(new Pose2d(deltaX + ballPose.getX(),
                        deltaY + ballPose.getY(),
                        ballPose.getRotation()));

                powercell.setLastTimestamp(currentTime);
                break;
            case 1:
                // Ball has been picked up by the robot
                powercell.setBallPose(m_field2d.getObject("Turret").getPose());

                // Ball has been shot;
                if(powercell.getBallShotState()) {
                    powercell.setBallShotState(false);
                    powercell.setLastTimestamp(RobotController.getFPGATime());
                    powercell.setBallState(2);
                    ballCount--;
                }
                break;
            case 0:
            default:
                if(isBallInIntakeZone(ballPose) && ballCount < 6) {
                    ballCount++;
                    powercell.setBallState(1);
                }
                break;
        }
    }
}