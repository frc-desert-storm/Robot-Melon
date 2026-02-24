package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class TurretRPMCommand extends Command {

  private Turret turret;
  private boolean bIsRight;

  public TurretRPMCommand(Turret turret, boolean bIsRight) {
    this.turret = turret;
    this.bIsRight = bIsRight;

    addRequirements(turret);
  }

  public void rotateRight(double rpm) {
    turret.rotateRight(rpm);
  }

  public void rotateLeft(double rpm) {
    turret.rotateLeft(rpm);
  }

  @Override
  public void initialize() {
    if (bIsRight) rotateRight(Constants.turretRPMRight);
    else rotateLeft(Constants.turretRPMLeft);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
