package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(Arm arm, Indexer indexer, Intake intake, Shooter shooter) {
        super(new IntakeNoteCommandGroup(intake, indexer), new SendBackCommandGroup(indexer, shooter).withTimeout(0));
    }
}
