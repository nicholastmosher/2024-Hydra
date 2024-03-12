package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeNoteCommandGroup extends ParallelDeadlineGroup {
    public IntakeNoteCommandGroup(Intake intake, Indexer indexer) {
        super(new IntakeNote(intake), new IndexNote(indexer));
    }
}
