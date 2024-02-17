package frc.robot.commands.Debug;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TestSwerve extends Command {
    private final Swerve mSwerve;
    private final double mX;
    private final double mY;
    private final double mRotation;
    public TestSwerve(Swerve swerve, double x, double y, double rotation) {
        mSwerve = swerve;
        mX = x;
        mY = y;
        mRotation = rotation;
    }

    @Override
    public void initialize() {
        mSwerve.drive(new Translation2d(mX,mY), mRotation,false,true);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        mSwerve.drive(new Translation2d(0,0),0, false, true);
    }
}
