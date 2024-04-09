package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShuffleNote extends SequentialCommandGroup {


    public ShuffleNote(Indexer indexer, Shooter shooter) {
        super(new IndexNote(indexer).withTimeout(0.3), new SendBackCommandGroup(indexer, shooter).withTimeout(0.4));
    }

}
