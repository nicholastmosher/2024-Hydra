package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.Debug;
import frc.lib.Constants.VisionParameters;
import frc.lib.config.VisionConfig;
import frc.robot.classes.Limelight.LimelightController;


public class Vision {

    private final LimelightController shootLimelight;
    private final LimelightController intakeLimelight;

    private final PIDController shootPID;
    private final PIDController intakePID;

    private double angleToNote;
    private double angleToShootAngle;

    private final VisionConfig config;

    public Vision(VisionConfig visionConfig) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(1, 0, 0);
        this.intakePID = new PIDController(0.01, 0, 0);
        this.angleToNote = 0.0;
        this.angleToShootAngle = 0.0;
    }

    public double getAngleToNote() {
        return angleToNote;
    }

    public double getAngleToShootAngle() {
        return angleToShootAngle;
    }

    public void periodic() {
        angleToNote = intakePID.calculate(intakeLimelight.getYawToNote(), 0);
        angleToShootAngle = shootPID.calculate(shootLimelight.getYawToSpeaker(), 0);
        SmartDashboard.putNumber("intakePID", angleToNote);
        SmartDashboard.putNumber("shootPID", angleToNote);
    }

}