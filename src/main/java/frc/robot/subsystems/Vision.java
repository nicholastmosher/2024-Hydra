package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.VisionConfig;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.classes.RollingAverage;


public class Vision {

    private final LimelightController shootLimelight;
    private final LimelightController intakeLimelight;

    private final PIDController shootPID;
    private final PIDController intakePID;
    private final RollingAverage intakeAverage = new RollingAverage(5);

    private double angleToNote;
    private double angleToShootAngle;

    private final VisionConfig config;

    public Vision(VisionConfig visionConfig) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(1, 0, 0);
//        this.intakePID = new PIDController(0.01, 0, 0);
//        this.intakePID = new CustomPid(0.25, 0.2, 0);
        this.intakePID = new PIDController(2.0, 0.01, .20);
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
        intakeAverage.addInput(intakeLimelight.getYawToNote());
        angleToNote = intakePID.calculate(intakeAverage.getOutput(), 0);
        angleToShootAngle = shootPID.calculate(shootLimelight.getYawToSpeaker(), 0);
        SmartDashboard.putNumber("intakePID", angleToNote);
        SmartDashboard.putNumber("shootPID", angleToNote);
    }

}