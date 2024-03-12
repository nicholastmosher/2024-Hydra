package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ArmConfig;
import frc.lib.config.DashboardConfig;

public class Arm extends SubsystemBase {
    private final CANSparkMax armRightMotor;
    private final CANSparkMax armLeftMotor;

    private final SparkPIDController armPID;

    private final AbsoluteEncoder armEncoder;

    private final DigitalInput armLimitSwitch;

    public final ArmConfig config;

    public final DashboardConfig dashboardConfig;

    public Arm(ArmConfig config, DashboardConfig dashboardConfig) {
        this.config = config;

        this.dashboardConfig = dashboardConfig;

        armRightMotor = new CANSparkMax(this.config.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armRightMotor.getPIDController();
        armEncoder = armRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        armEncoder.setPositionConversionFactor(config.positionScalingFactor);
        armEncoder.setZeroOffset(config.zeroOffset.getDegrees());
        armPID.setFeedbackDevice(armEncoder);
        armPID.setPositionPIDWrappingEnabled(false);
        armPID.setPositionPIDWrappingMaxInput(config.ampAngle.getDegrees() +5);
        armPID.setPositionPIDWrappingMinInput(0);
        armPID.setP(0.1);
        armPID.setI(1e-4);
        armPID.setD(0);
        armPID.setFF(0);

        armLeftMotor = new CANSparkMax(this.config.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armLeftMotor.follow(armRightMotor, true);
        armLeftMotor.getPIDController().setP(0.1);

        armLimitSwitch = config.armLimitSwitch;
    }

    public void init() {

    }

    public void setAngle(Rotation2d angle) {
        if (!isFullLower() ||  armEncoder.getVelocity() >0) {
            Rotation2d motorAngle =Rotation2d.fromRotations(armRightMotor.getEncoder().getPosition());
            System.out.println(getPosition().getDegrees());
            System.out.println(motorAngle.getDegrees());
            armRightMotor.getEncoder().setPosition(getPosition().getRotations());
            armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        } else {
            armRightMotor.stopMotor();
        }

    }

    public void moveArm(double input) {
        armRightMotor.set(input);

    }

    public boolean endCondition(Rotation2d angle) {
        return armEncoder.getPosition() > (angle.getDegrees() - 1) && armEncoder.getPosition() < (angle.getDegrees() + 1);
    }

    public void stopSet() {
        armRightMotor.stopMotor();
        armLeftMotor.stopMotor();
        System.out.println("Stop All Arm Movement");
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(armEncoder.getPosition());
    }

    private boolean isFullLower() {
        return !armLimitSwitch.get();
    }
}
