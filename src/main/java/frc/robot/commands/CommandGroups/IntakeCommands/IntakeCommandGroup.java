package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.commands.Indexer.SendBackIndexer;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Rumble.rumbleForOneSeconds;
import frc.robot.commands.Shooter.SendBackShooter;
import frc.robot.subsystems.*;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(Indexer indexer, Intake intake, Shooter shooter, Light light, CommandXboxController controller) {
        super(new IntakeNoteCommandGroup(intake, indexer), new ParallelDeadlineGroup(new rumbleForOneSeconds(controller).withTimeout(0.3), new InstantCommand(light::setRed), new SendBackCommandGroup(indexer, shooter).withTimeout(0))
        );
    }
}

//class Container {
//    public Command intakeCommand(Intake intake, Indexer indexer, Shooter shooter) {
//        Command intakeCommand = new ParallelDeadlineGroup(
//                new IntakeNote(intake),
//                new IndexNote(indexer)
//        );
//
//        Command sendBackCommand = new ParallelDeadlineGroup(
//                new SendBackShooter(shooter),
//                new SendBackIndexer(indexer)
//        );
//
//        return new SequentialCommandGroup(
//            intakeCommand, sendBackCommand
//        );
//    }
//}
