package frc.robot.commands.Debug;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * Command that assigns a specific speed to the drive motors of the swerve drive
 */
public class TestSwerveSteerMotors extends Command {
    private static final double DEFAULT_SPEED = 0.50;
    public final int mModule;
    private final double mPosition;
    private final Swerve mSwerve;

    public TestSwerveSteerMotors(Swerve swerve, int module) {
        this(swerve, module, DEFAULT_SPEED);
    }

    public TestSwerveSteerMotors(Swerve swerve, int module, double position) {
        this.mSwerve = swerve;
        this.mModule = module;
        this.mPosition = position;
    }

    public void initialize() {
        mSwerve.debugSetSteeringSpeed(mModule, mPosition);
    }

    public void execute() {
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        mSwerve.debugSetSteeringSpeed(mModule, 0.0);
    }
}
