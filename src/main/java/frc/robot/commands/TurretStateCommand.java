package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class TurretStateCommand extends Command {

  private Turret turret;
  private Turret.TurretGoal goal;

  public TurretStateCommand(Turret turret, Turret.TurretGoal goal) {
    this.turret = turret;
    this.goal = goal;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.setGoal(goal);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setGoal(Turret.TurretGoal.DISABLED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
