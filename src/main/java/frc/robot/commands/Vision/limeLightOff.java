package frc.robot.commands.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;



import frc.robot.subsystems.Vision;

public class limeLightOff extends Command {
  /** Creates a new limeLightOff. */
  private Vision m_vision;

  public limeLightOff(Vision l) {
    this.m_vision = l;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

//    m_vision.();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}