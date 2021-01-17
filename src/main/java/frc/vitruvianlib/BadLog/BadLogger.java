package frc.vitruvianlib.BadLog;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

import java.io.File;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;

public class BadLogger {
	private static BadLog logger;
	static boolean isRunning = false;
	static boolean isMatch = false;
	static String usbPath = "/media/sda1/4201Robot/Logs/";
    static String roboRioPath = "/home/lvuser/4201Robot/Logs/";
    static String  basePath, logPath, logName;
    public static double m_logStartTime;

    public static SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

    private RobotContainer m_robotContainer;
	/* This is a wrapper class for team 1014's BadLog logging framework. This is so that we can use their
	 * logging library, while maintaining the coding structure of our previous logging framework.
	 */
	public BadLogger(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }
    
    public void startLogger() {
        // Check if base path is a valid directory
        try {
            File baseDir = new File(usbPath);
            if(baseDir.isDirectory() && baseDir.exists() && baseDir.canWrite()) {
                System.out.println("VitruvianLogger Info: Base Path found! Logging to USB...");
                basePath = usbPath;
            } else
                basePath = roboRioPath;
        } catch(Exception e) {
            System.out.println("VitruvianLogger Error: USB Path not detected. Backing up to roboRIO path.");
            basePath = roboRioPath;
        }

        // Stop logging if its already running to avoid issues and to write logs to new directory
        if (isRunning && !isMatch)
            stopLogger();

        // Determine where to save logs
        DriverStation.MatchType matchType = DriverStation.getInstance().getMatchType();
        m_logStartTime = Timer.getFPGATimestamp();
        logPath = basePath;

        if (matchType == DriverStation.MatchType.None) {
            if (DriverStation.getInstance().isDisabled()) {
                logPath += "Disabled/";
            } else if (DriverStation.getInstance().isAutonomous()) {
                logPath += "Auto/";
            } else if (DriverStation.getInstance().isOperatorControl()) {
                logPath += "TeleOp/";
            } else {
                logPath += "UnsortedLog/";
            }

            // Use timestamp for log location
            Timestamp timestamp = new Timestamp(System.currentTimeMillis());
            logName = dateFormat.format(timestamp) + ".bag";
        } else {
        	// TODO: Debug this section - match logs aren't generated 
            isMatch = true;
            String eventString = DriverStation.getInstance().getEventName();
            logPath += eventString + "/";
            String matchNo = String.format("%02d", DriverStation.getInstance().getMatchNumber());

            if (matchType == DriverStation.MatchType.Practice) {
                logPath += "Practice/";
                logName = "P" + matchNo + ".bag";
            } else if (matchType == DriverStation.MatchType.Qualification) {
                logPath += "Qualification/";
                logName = "QM" + matchNo + ".bag";
            } else if (matchType == DriverStation.MatchType.Elimination) {
            	// TODO: How to determine elim match type (QF, SF, F)?
                logPath += "Elimination/";
                logName = "E" + matchNo + ".bag";
            } else {
                logPath += "UnknownMatch/" + matchNo + "/";
                logName = "UN" + matchNo + ".bag";
            }
            File dirPath = new File(logPath);

            if(!dirPath.exists() && !dirPath.isDirectory())
                dirPath.mkdirs();

            // Contingency if multiple matches exist (i.e. a match replay)
            File filePath = new File(logPath + logName);
            int counter = 1;
            while (filePath.exists() && counter < 99) {
                String newPath = logPath + logName.replace(".bag", String.format("%02d", counter++) + ".bag");
                filePath = new File(newPath);
            }
            logName = logName.replace(".bag", String.format("%02d", --counter) + ".bag");
            
            if (counter == 99)
                System.out.println("VitruvianLogger Error: Counter has reached 99. " +
                        "Will start overwriting logs");
        }
        
        logger = BadLog.init(logPath + logName);
        initTopics();
        logger.finishInitialization();
        isRunning = true;
    }

    private void initTopics() {
	    m_robotContainer.initializeLogTopics();
    }

    public void stopLogger() {
        isRunning = false;
        isMatch = false;
    }
    
    public void updateLogs() {
    	if(isRunning) {
	    	logger.updateTopics();
	    	logger.log();
    	}
    }
}
