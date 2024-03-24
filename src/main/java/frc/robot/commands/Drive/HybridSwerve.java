package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hybrid.BlendedSwerve;
import frc.robot.hybrid.SwerveVector;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;

public class HybridSwerve extends Command {
    private Swerve s_Swerve;
    private Vision s_Vision;

    private BlendedSwerve swerveInputs;
    private BooleanSupplier robotCentricSup;

    public HybridSwerve(
            Swerve s_Swerve,
            Vision s_Vision,
            BlendedSwerve inputs,
            BooleanSupplier robotCentricSup
    ) {
        this.s_Swerve = s_Swerve;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve);

        this.swerveInputs = inputs;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        SwerveVector driveOutputs = swerveInputs.solve();

        double x = MathUtil.applyDeadband(driveOutputs.x(), 0.1);
        double y = MathUtil.applyDeadband(driveOutputs.y(), 0.1);
        Translation2d translation = new Translation2d(x, y);
        double rotation = MathUtil.applyDeadband(driveOutputs.rot(), 0.2);

        /* Drive */
        s_Swerve.drive(
                translation,
                rotation,
                !robotCentricSup.getAsBoolean(),
                true
        );
    }
}