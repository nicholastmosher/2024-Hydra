package frc.robot.commands.CommandGroups.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.containers.RobotStateMachine;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {

    private final Intake IntakeSubsystem;
    private final Indexer IndexerSubsystem;
    private final RobotStateMachine stateMachine;


    public TeleopIntake(Intake intakeSubsystem, Indexer indexerSubsystem, RobotStateMachine robotStateMachine) {
        this.IntakeSubsystem = intakeSubsystem;
        this.IndexerSubsystem = indexerSubsystem;
        this.stateMachine = robotStateMachine;
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (IntakeSubsystem.finishedIntaking()) {
            stateMachine.setIntaking(false);
        }

        if (this.stateMachine.isIntaking()) {
            IntakeSubsystem.pickUpNote();
            IndexerSubsystem.indexNote();
        }



    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
