package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterRPMCommand extends Command {

  private final Shooter shooter;
  private final double rpm;

  public ShooterRPMCommand(Shooter shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setRPM(rpm);
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
