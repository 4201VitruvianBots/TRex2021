package frc.robot.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class SimulationReferencePose {
    static FieldSim m_fieldSim;

    public SimulationReferencePose(FieldSim fieldSim) {
        m_fieldSim = fieldSim;
    }

    public static Pose2d getRobotFieldPose() {
        return m_fieldSim.getRobotPose();
    }
}
