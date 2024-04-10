package frc.robot.classes.krakenFalcon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.CtreConfigs;
import frc.lib.config.SwerveModuleConfig;
import frc.lib.config.krakenTalonConstants;
import frc.lib.util.Conversions;


public class SwerveModuleKrakenFalcon implements SwerveModule {
    private final Rotation2d angleOffset;

    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANcoder angleEncoder;
    private final int mModule;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(krakenTalonConstants.Swerve.driveKS, krakenTalonConstants.Swerve.driveKV, krakenTalonConstants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public final CtreConfigs ctreConfigs;

    private int frameCount = 0;
    private static final int frameReset = 10;

    public SwerveModuleKrakenFalcon(CtreConfigs ctreConfigs, SwerveModuleConfig moduleConfig, int moduleNumber) {
        this.ctreConfigs = ctreConfigs;
        this.angleOffset = moduleConfig.angleOffset;
        this.mModule = moduleNumber;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConfig.cancoderID);
        angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConfig.angleMotorID);
        mAngleMotor.setInverted(moduleConfig.angleInvert);
        mAngleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConfig.driveMotorID);
        mDriveMotor.setInverted(moduleConfig.driveInvert);
        mDriveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
//        if (frameCount % frameReset == 0) {
//            resetToAbsolute();
//            frameCount = 0;
//        }
        //frameCount++;
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()).withEnableFOC(true));
        setSpeed(desiredState, isOpenLoop);
    }

    @Override
    public void debugSetDriveSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    @Override
    public void debugSetSteeringSpeed(double speed) {
        mAngleMotor.set(speed);
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(
//                angleEncoder.getPosition().getValue()
                angleEncoder.getAbsolutePosition().getValue()
        );
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = getRotation().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
        System.out.printf("Absolute Position: %f\n", absolutePosition);
        System.out.printf("Cancoder: %f\n", getRotation().getRotations());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), krakenTalonConstants.Swerve.driveTrainConfig.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), krakenTalonConstants.Swerve.driveTrainConfig.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / krakenTalonConstants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle.withEnableFOC(true));
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, krakenTalonConstants.Swerve.driveTrainConfig.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity.withEnableFOC(true));
        }
    }

    private Rotation2d cancoderOffset = new Rotation2d();
    private Rotation2d motorOffset = new Rotation2d();
    public void zeroEncoders() {
        cancoderOffset = getRotation();
        motorOffset = Rotation2d.fromRotations(mAngleMotor.getPosition().getValue());
    }

    @Override
    public void dashboardPeriodic() {
        SmartDashboard.putNumber(String.format("CanCoder%dAngle", mModule), getRotation().getRotations());
        SmartDashboard.putNumber(String.format("MotorSteer%dAngle", mModule), Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()).getRotations());

        Rotation2d adjustedCancoder = Rotation2d.fromRotations(getRotation().getRotations() - cancoderOffset.getRotations());
        Rotation2d adjustedMotor = Rotation2d.fromRotations(mAngleMotor.getPosition().getValue() - motorOffset.getRotations());

        SmartDashboard.putNumber(String.format("ZeroCanCoder%dAngle", mModule), adjustedCancoder.getRotations());
        SmartDashboard.putNumber(String.format("ZeroMotor%dAngle", mModule), adjustedMotor.getRotations());

//        SmartDashboard.putNumber(String.format("DriveMotor%d Voltage", mModule), mDriveMotor.getMotorVoltage().getValue());
//        SmartDashboard.putNumber(String.format("AngleMotor%d Voltage", mModule), mAngleMotor.getMotorVoltage().getValue());
    }
}
