package frc.robot.commands.Shooter;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {
    Shooter shooter;
    CommandXboxController controller;

    public RevShooter(Shooter subsystem, CommandXboxController commandXboxController) {
        shooter = subsystem;
        this.controller = commandXboxController;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.startShooter();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
    }

    @Override
    public boolean isFinished() {
        if (shooter.isRevved()) {
            controller.getHID().setRumble(RumbleType.kRightRumble, 0.5);
        }
        return false;
    }

}
