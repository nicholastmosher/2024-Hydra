package frc.robot.commands.Drive;

import frc.lib.config.krakenTalonConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private Vision s_Vision;
    private DoubleSupplier y;
    private DoubleSupplier x;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier autoAimSup;
    private BooleanSupplier autoIntakeAlignSup;

    public TeleopSwerve(
            Swerve s_Swerve,
            Vision s_Vision,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier autoAimSup,
            BooleanSupplier autoIntakeAlignSup
    ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.y = y;
        this.x = x;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.autoAimSup = autoAimSup;
        this.autoIntakeAlignSup = autoIntakeAlignSup;
        this.s_Vision = s_Vision;
    }

    @Override
    public void execute() {
        double maxSpeed;

        /* Get Values, Deadband*/
        double y = MathUtil.applyDeadband(this.y.getAsDouble(), krakenTalonConstants.stickDeadband);
        double x = MathUtil.applyDeadband(this.x.getAsDouble(), krakenTalonConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), krakenTalonConstants.stickDeadband);

        if (autoAimSup.getAsBoolean() && !autoIntakeAlignSup.getAsBoolean()) {
            rotationVal = MathUtil.applyDeadband(s_Vision.getAngleToShootAngle(), 0.1);
        }

        if (autoIntakeAlignSup.getAsBoolean()) {
            rotationVal = MathUtil.applyDeadband(s_Vision.getNoteAimRotationPower(), 0.1);
        }

        Translation2d translation = new Translation2d(x*krakenTalonConstants.Swerve.maxSpeed, y*krakenTalonConstants.Swerve.maxSpeed);
        Translation2d dummytranslation = new Translation2d(0, 0);

        /* Drive */
        s_Swerve.drive(
                translation,
                rotationVal, //* krakenTalonConstants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
        );

        //SmartDashboard.putNumber("shootorient",shootOrient);
//        SmartDashboard.putNumber("", );
    }
}