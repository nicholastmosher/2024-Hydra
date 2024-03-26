package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ClimberConfig;
import frc.lib.config.DashboardConfig;

public class Climber extends SubsystemBase {
    private final TalonFX rightClimberMotor;
    private final TalonFX leftClimberMotor;
    private final double climberSpeed;
    private final double downclimberSpeed;
    private final DigitalInput rightLimitSwitch;
    private final DigitalInput leftLimitSwitch;

    public final ClimberConfig config;
    public final DashboardConfig dashConfig;


    public Climber(ClimberConfig climberConfig, DashboardConfig dashboardConfig) {
        this.config = climberConfig;
        this.dashConfig = dashboardConfig;

        this.rightClimberMotor = new TalonFX(config.rightClimberMotorID);
        this.leftClimberMotor = new TalonFX(config.leftClimberMotorID);
//        leftClimberMotor.setControl(new Follower(config.rightClimberMotorID, true));
        this.climberSpeed = config.climberSpeed;
        this.downclimberSpeed = config.downclimberSpeed;

        this.rightLimitSwitch = config.rightMotorLimitSwitch;
        this.leftLimitSwitch = config.leftMotorLimitSwitch;
    }

    public void climberExtend(){
        rightClimberMotor.set(this.climberSpeed);
        leftClimberMotor.set(this.climberSpeed);
    }

    public void climberRetract(){
        rightClimberMotor.set(this.downclimberSpeed);
        leftClimberMotor.set(this.downclimberSpeed);
    }

    public void stop(){
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    private boolean isLeftFullLower() {
        if (!leftLimitSwitch.get()) {
            leftClimberMotor.setPosition(0);
            return true;
        }


        return false;
    }
    private boolean isRightFullLower() {
        if (!rightLimitSwitch.get()) {
            rightClimberMotor.setPosition(0);
            return true;
        }


        return false;


    }

    public void init() {
        while (!isRightFullLower()) {
            rightClimberMotor.set(downclimberSpeed);
        }
        rightClimberMotor.stopMotor();
        rightClimberMotor.setPosition(0);
        while (!isLeftFullLower()) {
            leftClimberMotor.set(downclimberSpeed);
        }
        leftClimberMotor.stopMotor();
//        leftClimberMotor.setPosition(leftClimberMotor.getPosition().getValue());

        leftClimberMotor.setControl(new Follower(config.rightClimberMotorID, false));
    }

    public void joystickControl(double Input) {
        rightClimberMove(Input);
        leftClimberMove(Input);
    }

    // Assume positive is up and negative is down
    private void rightClimberMove(double in) {
        if (in > 0) {
            if (isRightFullLower()) {
                rightClimberMotor.set(0);
                return;
            }

            rightClimberMotor.set(in);
            return;
        }
        if (in < 0) {
            rightClimberMotor.set(in);
            return;
        }
        rightClimberMotor.set(0);





    }

    private void leftClimberMove(double in) {
        if (in > 0) {
            if (isLeftFullLower()) {
                leftClimberMotor.set(0);
                return;
            }

            leftClimberMotor.set(in);
            return;
        }
        if (in < 0) {
            leftClimberMotor.set(in);
            return;
        }
        leftClimberMotor.set(0);
    }

    @Override
    public void periodic() {
//        SmartDashboard.putBoolean("isLeftFullLower", isLeftFullLower());
        // SmartDashboard.putBoolean("isRightFullLower", isRightFullLower());
        // SmartDashboard.putBoolean("isLeftFullLower", isLeftFullLower());
        // SmartDashboard.putNumber("rightclimberencoder", rightClimberMotor.getPosition().getValue());
        // SmartDashboard.putNumber("rightclimberencoder", leftClimberMotor.getPosition().getValue());
    }


}
