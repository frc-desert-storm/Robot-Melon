package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterRPMCommand extends Command {

  private Shooter shooter;
  private boolean bIsForwards;

  public ShooterRPMCommand(Shooter shooter, boolean bIsForwards) {
    this.shooter = shooter;
    this.bIsForwards = bIsForwards;

     addRequirements(shooter);
  }

  public void setRPM(double rpm) {
    shooter.setRPM(rpm);
  }

  @Override
  public void initialize() {
    setRPM(bIsForwards ? Constants.shooterForwardRPM : Constants.shooterBackwardRPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
