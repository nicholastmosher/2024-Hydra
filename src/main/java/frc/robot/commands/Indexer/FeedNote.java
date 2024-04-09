package frc.robot.commands.Indexer;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FeedNote extends Command {
    Indexer indexer;
    CommandXboxController controller;

    public FeedNote(Indexer subsystem, CommandXboxController commandXboxController) {
        indexer = subsystem;
        this.controller = commandXboxController;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.feedNote();
        controller.getHID().setRumble(RumbleType.kRightRumble, 0);
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
