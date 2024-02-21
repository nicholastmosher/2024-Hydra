package frc.robot.commands.Drive.debug;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class DiagnoseSwerveRotation extends SequentialCommandGroup {
    public DiagnoseSwerveRotation(Swerve swerve){
        super(
                new TestSwerve(swerve, 0.0,0.0,1.0).withTimeout(2.0)
        );
    }
}
