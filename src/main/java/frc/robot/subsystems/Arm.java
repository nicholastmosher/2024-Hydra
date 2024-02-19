package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final CANSparkMax armFollowerMotor;

    private final SparkPIDController armPID;

    private final AbsoluteEncoder armEncoder;

    public Arm() {
        armMotor = new CANSparkMax(Constants.Arm.armMotor, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armMotor.getPIDController();
        configArmMotor();

        armFollowerMotor = new CANSparkMax(Constants.Arm.armFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configFollowerMotor();

        armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    }

    public void setAngle(Rotation2d angle) {
        armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

    public boolean endConditionIntake() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredIntakeAngle);
        return armEncoder.getPosition() > (angle.getDegrees() + 1) && armEncoder.getPosition() < (angle.getDegrees() - 1);
    }

    public boolean endConditionShoot() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredShooterAngle);
        return armEncoder.getPosition() > (angle.getDegrees() + 1) && armEncoder.getPosition() < (angle.getDegrees() - 1);
    }

    public void stopSet() {
        armMotor.stopMotor();
    }

    private void configArmMotor() {
        armPID.setOutputRange(Constants.Arm.minAngle, Constants.Arm.maxAngle);
        armPID.setFeedbackDevice(armEncoder);
    }

    private void configFollowerMotor() {
        armFollowerMotor.follow(armMotor, true);
    }
}
