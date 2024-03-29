package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.commands.Indexer.SendBackIndexer;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.SendBackShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(Indexer indexer, Intake intake, Shooter shooter) {
        super(new IntakeNoteCommandGroup(intake, indexer), new SendBackCommandGroup(indexer, shooter).withTimeout(0));
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
