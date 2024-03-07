package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;


public class Retract extends Command {
    private final Climber climber;

    public Retract(Climber climber) {
        this.climber = climber;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climber.climberRetract();
        System.out.println("Retracting");
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {climber.stop();}
}
