package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.Debug;
import frc.lib.Constants.VisionParameters;


public class Vision extends SubsystemBase {
    NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry m_tx = m_table.getEntry("tx");
    NetworkTableEntry m_ty = m_table.getEntry("ty");
    NetworkTableEntry m_ta = m_table.getEntry("ta");
    NetworkTableEntry m_tid = m_table.getEntry("tid");
    NetworkTableEntry m_ledMode = m_table.getEntry("ledMode");
    NetworkTableEntry m_pipeLine = m_table.getEntry("pipeline");
    NetworkTableEntry m_llpython = m_table.getEntry("llpython");

    private double m_llpythonReturn[];
    private boolean m_lightOn = false;

    private double m_currentPipeline = VisionParameters.k_aprilTagPipeline;

    /**
     * Creates a new vision.
     */
    public Vision() {

        off();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double x = m_tx.getDouble(0.0);
        double y = m_ty.getDouble(0.0);
        double id = m_tid.getDouble(0.0);
        double area = m_ta.getDouble(0.0);


        if (Debug.VisionON) {
            SmartDashboard.putNumber("LimelightX", x);
            SmartDashboard.putNumber("LimelightY", y);
            SmartDashboard.putNumber("LimelightArea", area);
            SmartDashboard.putNumber("AprilTag ID;", id);
        }


        double ledState = m_ledMode.getDouble(-1);
        if (Debug.VisionON) {
            SmartDashboard.putNumber("ledMode", ledState);
        }


        m_pipeLine.setNumber(m_currentPipeline);
        if (Debug.VisionON) {
            SmartDashboard.putNumber("Pipe", m_currentPipeline);
        }

        if (!m_lightOn) {
            off();
        }


    }

    public void off() {
        m_ledMode.setNumber(VisionParameters.k_lightOff);
        m_lightOn = false;
    }

    public void on() {
        m_ledMode.setNumber(VisionParameters.k_lightOn);
        m_lightOn = true;
    }

    public void swapPipeline() {
        if (m_currentPipeline == VisionParameters.k_aprilTagPipeline) {
            m_currentPipeline = VisionParameters.k_retrotapePipeline;
            on();
        } else {
            m_currentPipeline = VisionParameters.k_aprilTagPipeline;
            off();
        }
    }


    public boolean targetFound() {
        boolean found = false;
        double id = m_tid.getDouble(0.0);
        if (id != -1) {
            if (Math.abs(m_tx.getDouble(10)) <= VisionParameters.k_xTargetBounds) {
                found = true;
            }
        }
        return (found);
    }

    public boolean targetFoundLeft() {
        boolean found = false;
        double id = m_tid.getDouble(0.0) + VisionParameters.k_offset;
        if (id != -1) {
            if (Math.abs(m_tx.getDouble(10)) <= VisionParameters.k_xTargetBounds) {
                found = true;
            }
        }
        return (found);
    }

    public boolean targetFoundRight() {
        boolean found = false;
        double id = m_tid.getDouble(0.0) - VisionParameters.k_offset;
        if (id != -1) {
            if (Math.abs(m_tx.getDouble(10)) <= VisionParameters.k_xTargetBounds) {
                found = true;
            }
        }
        return (found);
    }

    /**
     * Gets the vision returns from the pipeline selected
     *
     * @param pipe - the pipeline to read from
     * @return - array of vision data, tx, ty and ta
     */
    public double[] findObject() {
        double targetInfo[] = new double[3];

        targetInfo[0] = m_tx.getDouble(0.0);
        targetInfo[1] = m_ty.getDouble(0.0);
        targetInfo[2] = m_ta.getDouble(0.0);

        return (targetInfo);
    }

    public double getPipeline() {
        return (m_currentPipeline);
    }

    /**
     * determine if the target relecting tape is to the left
     */
    //TODO flesh this out
    public boolean markerToLeft() {
        return true;
    }

    /**
     * determine if the target relecting tape is in front of robot
     */
    //TODO flesh this out
    public boolean markerFound() {
        return true;
    }

}