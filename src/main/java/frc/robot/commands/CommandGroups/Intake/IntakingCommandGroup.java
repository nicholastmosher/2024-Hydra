package frc.robot.commands.CommandGroups.Intake;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.IndexNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakingCommandGroup extends ParallelDeadlineGroup {
    public IntakingCommandGroup(Intake intake, Shooter shooter) {
        super(new IntakeNote(intake), new IndexNote(shooter));
    }
}