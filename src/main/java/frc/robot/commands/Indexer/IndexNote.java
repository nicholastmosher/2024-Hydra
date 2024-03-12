package frc.robot.commands.Indexer;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IndexNote extends Command {
    Indexer indexer;

    public IndexNote(Indexer subsystem) {
        indexer = subsystem;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.indexNote();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
