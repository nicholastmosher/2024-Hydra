package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModuleNeoNeo implements SwerveModule {

    private Rotation2d angleOffset;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANcoder angleEncoder;
    private SwerveModuleState mCurrentState = new SwerveModuleState(0, new Rotation2d(0));

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    // private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    // private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    // private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModuleNeoNeo(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        //mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        //mDriveMotor.getConfigurator().setPosition(0.0);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        SparkPIDController pidController = mAngleMotor.getPIDController();
        pidController.setReference(desiredState.angle.getRotations(),CANSparkMax.ControlType.kPosition);
        //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    @Override
    public Rotation2d getRotation(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getRotation().getRotations() - angleOffset.getRotations();

        SparkPIDController pidController = mAngleMotor.getPIDController();
        pidController.setReference(absolutePosition,CANSparkMax.ControlType.kPosition);

        //mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return mCurrentState;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mDriveMotor.getEncoder().getPosition())
        );
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        mCurrentState = desiredState;

        if(isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(speed);

            // driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            //mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            // driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            double voltage = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            // mDriveMotor.setControl(driveVelocity);
            mDriveMotor.setVoltage(voltage);
        }
    }
}