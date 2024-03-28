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

    private final VisionConfig config;

    public Vision(VisionConfig visionConfig) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(1.25, 0.01, 0.2);
//        this.intakePID = new PIDController(0.01, 0, 0);
//        this.intakePID = new CustomPid(0.25, 0.2, 0);
        this.intakePID = new PIDController(1.5, 0.01, .30); //use to be kp =2
        this.aimRotationPower = 0.0;
        this.shootDistancePID = new PIDController(2.4, 0, 0);       
        this.angleToShootAngle = 0.0;
        this.autoApproachPower = 0.50;
        this.limelightMountAngleDegrees = 37.0;
        this.limelightMountHeightInches = 10.0;
        this.goalHeightInches = 64.0;
    }

    /**
     * The output of a PID loop, from -1.0 to 1.0, describing
     * how hard the swerve should rotate to aim at the target
     */
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
        intakeAverage.addInput(intakeLimelight.getYawToNote());
        shootAverage.addInput(shootLimelight.getYawToSpeaker());
        shootDistAverage.addInput(shootLimelight.distanceToSpeaker());

        angleToGoalRadians = (limelightMountAngleDegrees + shootDistAverage.getOutput()) * (3.14159 / 180.0);
        distanceFromLimelightToSpeakerInches = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoalRadians);

        aimRotationPower = intakePID.calculate(intakeAverage.getOutput(), 0);
        angleToShootAngle = shootPID.calculate(shootAverage.getOutput(), 0);
        autoApproachPower = shootDistancePID.calculate(shootDistAverage.getOutput(),1.3);
        SmartDashboard.putNumber("intakePID", aimRotationPower);
        SmartDashboard.putNumber("shootPID", aimRotationPower);
        SmartDashboard.putNumber("shootDistance", shootDistAverage.getOutput()); //subwofer flush 1.35 community 0.75


    }
}