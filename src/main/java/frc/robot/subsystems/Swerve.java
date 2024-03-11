package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.CtreConfigs;
import frc.lib.Constants;
import frc.lib.krakentalon.krakenTalonConstants;
import frc.robot.interfaces.SwerveModule;
import frc.robot.classes.SwerveModuleKrakenFalcon;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private ChassisSpeeds latestSpeeds;

    /**
     * Default constructor uses SwerveModuleTalonNeo
     */
    public Swerve(CtreConfigs ctreConfigs) {
        this(new SwerveModule[]{
                new SwerveModuleKrakenFalcon(ctreConfigs, Constants.mod3backrightConfig, 3),
                new SwerveModuleKrakenFalcon(ctreConfigs, Constants.mod1frontrightConfig, 1),
                new SwerveModuleKrakenFalcon(ctreConfigs, Constants.mod2backleftConfig, 2),
                new SwerveModuleKrakenFalcon(ctreConfigs, Constants.mod0frontleftConfig, 0)
        });
    }

    /**
     * Constructor that allows custom SwerveModules
     */
    public Swerve(SwerveModule[] modules) {
        this.mSwerveMods = modules;
        gyro = new Pigeon2(krakenTalonConstants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        swerveOdometry = new SwerveDriveOdometry(krakenTalonConstants.Swerve.driveTrainConfig.kinematics, getGyroYaw(), getModulePositions());
        this.latestSpeeds = new ChassisSpeeds(0, 0,0);
    }

    public ChassisSpeeds getLatestSpeeds() {
        return latestSpeeds;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading()
        ) : new ChassisSpeeds(
                -translation.getX(),
                -translation.getY(),
                rotation);

        driveChassisSpeeds(speeds, isOpenLoop);
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        this.latestSpeeds = speeds;
        SmartDashboard.putNumber("ChassisSpeedX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedY", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedTheta", speeds.omegaRadiansPerSecond);
        SwerveModuleState[] swerveModuleStates = krakenTalonConstants.Swerve.driveTrainConfig.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, krakenTalonConstants.Swerve.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    public void debugSetDriveSpeed(int module, double speed) {
        mSwerveMods[module].debugSetDriveSpeed(speed);
    }

    public void debugSetSteeringSpeed(int module, double speed) {
        mSwerveMods[module].debugSetSteeringSpeed(speed);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, krakenTalonConstants.Swerve.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(desiredStates[i], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            states[i] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            positions[i] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        Pose2d pose = swerveOdometry.getPoseMeters();
        SmartDashboard.putNumber("SwervePoseX", pose.getX());
        SmartDashboard.putNumber("SwervePoseY", pose.getY());
        SmartDashboard.putNumber("SwervePoseTheta", pose.getRotation().getDegrees());
        return pose;
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(getGyroYaw().getDegrees()+180), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.resetToAbsolute();
        }
    }

    public void zeroEncoders(){
        for(SwerveModule mod : mSwerveMods){
            mod.zeroEncoders();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        // for (int i = 0; i < mSwerveMods.length; i++) {
        //     SwerveModule mod = mSwerveMods[i];
        //     mod.dashboardPeriodic();
        // }
    }
}