package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Indexer.SendBackIndexer;
import frc.robot.commands.Shooter.SendBackShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SendBackCommandGroup extends ParallelCommandGroup {
    public SendBackCommandGroup(Indexer indexer, Shooter shooter) {
        super(new SendBackShooter(shooter), new SendBackIndexer(indexer));
    }

}
