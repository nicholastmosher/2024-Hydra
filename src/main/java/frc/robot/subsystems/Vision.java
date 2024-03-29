package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.VisionConfig;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.classes.RollingAverage;


public class Vision {

    private final LimelightController shootLimelight;
    private final LimelightController intakeLimelight;
    private final PIDController shootPID;
    private final PIDController intakePID;
    private final PIDController shootDistancePID;
    private final RollingAverage intakeAverage = new RollingAverage(5);
    private final RollingAverage shootAverage = new RollingAverage(5);
    private final RollingAverage shootDistAverage = new RollingAverage(5);
    private double aimRotationPower;
    private double angleToShootAngle;
    private double autoApproachPower;

    private double limelightMountAngleDegrees;
    private double limelightMountHeightInches;
    private double goalHeightInches;


    private double shootsetpoint = 0.0;
    private final VisionConfig config;

    boolean isBlue;

    public Vision(VisionConfig visionConfig, DriverStation.Alliance alliance) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(1.125, 0.01, 0.3);
        this.intakePID = new PIDController(1.5, 0.01, .30); //use to be kp =2
        this.shootDistancePID = new PIDController(.075, 0.01, 0.01);
      
        this.aimRotationPower = 0.0;
        this.angleToShootAngle = 0.0;
        this.autoApproachPower = 0.0;
      
        this.limelightMountAngleDegrees = 37.0;
        this.limelightMountHeightInches = 10.0;
        this.goalHeightInches = 67.0;

        if (alliance == DriverStation.Alliance.Blue) {
            isBlue = true;
        } else {
            isBlue = false;
        }
    }

    public double getNoteAimRotationPower() {
        return aimRotationPower;
    }

    public double getAngleToShootAngle() {
        return angleToShootAngle;
    }

    public double getAutoApproachPower(){
        return autoApproachPower;
    }

    public void periodic() {
        double angleToGoalRadians = (limelightMountAngleDegrees + shootLimelight.distanceToSpeaker()) * (3.14159 / 180.0);
        double distanceFromLimelightToSpeakerInches = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoalRadians);
        
      shootsetpoint=0.0;
        if (shootLimelight.tagsSeen() >= 2) {
            if (isBlue) {
                shootsetpoint = -0.5;
            } else {
                shootsetpoint = 0.5;
            }
        }
        
        intakeAverage.addInput(intakeLimelight.getYawToNote());
        shootAverage.addInput(shootLimelight.getYawToSpeaker());
        shootDistAverage.addInput(distanceFromLimelightToSpeakerInches);

        aimRotationPower = intakePID.calculate(intakeAverage.getOutput(), 0);
        autoApproachPower = -shootDistancePID.calculate(shootDistAverage.getOutput(), 55);
        angleToShootAngle = shootPID.calculate(shootAverage.getOutput(), shootsetpoint);
    }
}