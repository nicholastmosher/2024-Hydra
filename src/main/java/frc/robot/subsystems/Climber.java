package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
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
        this.climberSpeed = config.climberSpeed;
        this.downclimberSpeed = config.downclimberSpeed;

        this.rightLimitSwitch = config.rightMotorLimitSwitch;
        this.leftLimitSwitch = config.leftMotorLimitSwitch;
    }

    public void init() {
        while (!rightLimitSwitch.get()) {
            rightClimberMotor.set(downclimberSpeed);
        }
        rightClimberMotor.stopMotor();
        rightClimberMotor.setPosition(rightClimberMotor.getPosition().getValue());
        while (!leftLimitSwitch.get()) {
            leftClimberMotor.set(downclimberSpeed);
        }
        leftClimberMotor.stopMotor();
        leftClimberMotor.setPosition(leftClimberMotor.getPosition().getValue());
        leftClimberMotor.setControl(new Follower(config.rightClimberMotorID, false));
    }

    public void joystickControl(double input) {
        rightClimberMotor.set(input*input);
    }


}
