package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.VisionConfig;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.classes.RollingAverage;
import pabeles.concurrency.IntOperatorTask;


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


    private final double SubwooferDist = 37.9;

    private final double halfwayredLine = 49.1;

    private final double MinDist = 74;

    private final double MaxDist = 105;


    private final double cutOffDist = 74;

    private double angleToGoalRadians;
   private double distanceFromLimelightToSpeakerInches;

    private final double minAngleToShoot =0.4785;
    private final double maxAngleToShoot =0.5025;


    public Vision(VisionConfig visionConfig, DriverStation.Alliance alliance) {
        this.config = visionConfig;
        this.shootLimelight = new LimelightController(config.shootLimelightName);
        this.intakeLimelight = new LimelightController(config.intakeLimelightName);
        this.shootPID = new PIDController(0.75, 0.00, 0.01);
        this.intakePID = new PIDController(0.9, 0.05, .01); //use to be kp =2
        this.shootDistancePID = new PIDController(.075, 0.00, 0.01);
      
        this.aimRotationPower = 0.0;
        this.angleToShootAngle = 0.0;
        this.autoApproachPower = 0.0;
      
        this.limelightMountAngleDegrees = 37.0;
        this.limelightMountHeightInches = 10.0;
        this.goalHeightInches = 57.0;

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
        if (distanceFromLimelightToSpeakerInches > cutOffDist) {
            return 0;
        }
        autoApproachPower = -shootDistancePID.calculate(shootDistAverage.getOutput(), (SubwooferDist+halfwayredLine)/2);

        return autoApproachPower;
    }

    public double getArmAngleForShoot() {
        if (getAutoApproachPower() >0) {
            return 0.44;
        }
        double x1 = MinDist;
        double x2 = MaxDist;
        double y1 = minAngleToShoot;
        double y2 = maxAngleToShoot;

        double m = (y2-y1)/(x2-x1);
        double b = y1-(m*x1);

        double x = distanceFromLimelightToSpeakerInches;
//        double m = 0.00070968;
        double mx = m*x;
//        double b = 0.42548368;
        double y = mx +b;
        double angle = y;

//        double distFromMin = distanceFromLimelightToSpeakerInches- MinDist;
//        double maxDistDiff = MaxDist- MinDist;
//        double percentBack = distFromMin/maxDistDiff;
//
//        double MaxArmDiff = MaxDist - MinDist;
//
//
//        double angle = MaxArmDiff/percentBack;


        SmartDashboard.putNumber("armPower", angle);
        return angle;


    }


    public void periodic() {
        angleToGoalRadians = (limelightMountAngleDegrees + shootLimelight.distanceToSpeaker()) * (3.14159 / 180.0);
        distanceFromLimelightToSpeakerInches = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoalRadians);
        SmartDashboard.putNumber("noteAngle", intakeLimelight.getYawToNote());
        
      shootsetpoint=0.0;
//        if (shootLimelight.tagsSeen() >= 2) {
//            if (isBlue) {
//                shootsetpoint = -0.5;
//            } else {
//                shootsetpoint = -0.3;
//            }
//        }
        
        intakeAverage.addInput(intakeLimelight.getYawToNote());
        shootAverage.addInput(shootLimelight.getYawToSpeaker());
        shootDistAverage.addInput(distanceFromLimelightToSpeakerInches);

        aimRotationPower = intakePID.calculate(intakeAverage.getOutput(), 0);

        angleToShootAngle = shootPID.calculate(shootAverage.getOutput(), shootsetpoint);
    }
}