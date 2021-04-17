package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryUtils {
    public static ArrayList<Pose2d> readCsvTrajectory(String filename) {
        BufferedReader reader;
        String fileLine;
        String[] fields;
        ArrayList<Pose2d> trajectoryPoints = new ArrayList<>();
        String fullpath = "/home/lvuser/deploy/Trajectories/" + filename + ".csv";
        try {
            reader = new BufferedReader(new FileReader(fullpath));
            while ((fileLine = reader.readLine()) != null) {
                fields = fileLine.split(",");
                trajectoryPoints.add(new Pose2d(Units.feetToMeters(Double.parseDouble(fields[0])),
                                                Units.feetToMeters(Double.parseDouble(fields[1])),
                                                Rotation2d.fromDegrees(Double.parseDouble(fields[2]))));

            }
        } catch (FileNotFoundException e) {
            System.out.println("Error: Could not find file");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("Error: Could not read file");
            e.printStackTrace();
        }
        return trajectoryPoints;
    }

//    public static VitruvianRamseteCommand generateRamseteCommand(DriveTrain driveTrain, ArrayList<Pose2d> path, TrajectoryConfig config) {
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, config);
//
//        VitruvianRamseteCommand ramseteCommand = new VitruvianRamseteCommand(
//                trajectory,
//                driveTrain::getRobotPose,
//                new RamseteController(),
//                driveTrain.getFeedforward(),
//                driveTrain.getDriveTrainKinematics(),
//                driveTrain::getSpeeds,
//                driveTrain.getLeftPIDController(),
//                driveTrain.getRightPIDController(),
//                driveTrain::setVoltageOutput,
//                driveTrain,
//                path,
//                config
//        );
//        return ramseteCommand;
//    }

    public static Trajectory generateSwerveTrajectory(TrajectoryConfig config, int[][] waypoints) {
        Pose2d start = new Pose2d(
                Units.inchesToMeters(waypoints[0][0]),
                Units.inchesToMeters(waypoints[0][1]),
                new Rotation2d(Units.degreesToRadians(waypoints[0][2]))
        );

        Pose2d end = new Pose2d(
                Units.inchesToMeters(waypoints[waypoints.length - 1][0]),
                Units.inchesToMeters(waypoints[waypoints.length - 1][1]),
                new Rotation2d(Units.degreesToRadians(waypoints[waypoints.length - 1][2]))
        );

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        for (int i = 1; i < waypoints.length - 1; i++) {
            internalPoints.add(new Translation2d(
                    Units.inchesToMeters(waypoints[i][0]),
                    Units.inchesToMeters(waypoints[i][1])
            ));
        }

        return TrajectoryGenerator.generateTrajectory(start, internalPoints, end, config);
    }
}
