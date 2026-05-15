package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Turret turret;
  private final Indexer indexer;
  private final Intake intake;

  @AutoLogOutput private SuperstructureState state = SuperstructureState.IDLE;

  @AutoLogOutput
  public final Trigger activeHubTrigger =
      new Trigger(() -> HubShiftUtil.getShiftedShiftInfo().active());

  @AutoLogOutput public final Trigger underTrenchTrigger;

  public Superstructure(
      Turret turret,
      Indexer indexer,
      Intake intake,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.turret = turret;
    this.indexer = indexer;
    this.intake = intake;

    underTrenchTrigger =
        Zones.TRENCH_DUCK_ZONES.willContain(poseSupplier, chassisSpeedsSupplier, DUCK_TIME);

    underTrenchTrigger.and(DriverStation::isTeleop).whileTrue(duck());
  }

  public enum SuperstructureState {
    IDLE,
    INTAKING,
    SCORING_WINDUP,
    SCORING,
    PASSING_WINDUP,
    PASSING,
    REVERSING,
    DUCKING,
  }

  private void applyState(SuperstructureState newState) {
    state = newState;
    switch (newState) {
      case IDLE -> {
        turret.setGoal(Turret.TurretGoal.IDLE);
        indexer.setState(Indexer.State.IDLE);
        intake.setState(Intake.PivotState.DOWN, Intake.RollerState.IDLE);
      }
      case INTAKING -> {
        turret.setGoal(Turret.TurretGoal.IDLE);
        indexer.setState(Indexer.State.FEEDING);
        intake.setState(Intake.PivotState.DOWN, Intake.RollerState.INTAKING);
      }
      case SCORING_WINDUP -> {
        turret.setGoal(Turret.TurretGoal.SCORING);
        indexer.setState(Indexer.State.IDLE);
        intake.setState(Intake.PivotState.UP, Intake.RollerState.IDLE);
      }
      case SCORING -> {
        turret.setGoal(Turret.TurretGoal.SCORING);
        indexer.setState(Indexer.State.SCORING);
        intake.setState(Intake.PivotState.UP, Intake.RollerState.INTAKING);
      }
      case PASSING_WINDUP -> {
        turret.setGoal(Turret.TurretGoal.PASSING);
        indexer.setState(Indexer.State.IDLE);
        intake.setState(Intake.PivotState.UP, Intake.RollerState.IDLE);
      }
      case PASSING -> {
        turret.setGoal(Turret.TurretGoal.PASSING);
        indexer.setState(Indexer.State.SCORING);
        intake.setState(Intake.PivotState.UP, Intake.RollerState.INTAKING);
      }
      case REVERSING -> {
        turret.setGoal(Turret.TurretGoal.IDLE);
        indexer.setState(Indexer.State.REVERSE);
        intake.setState(Intake.PivotState.DOWN, Intake.RollerState.REVERSE);
      }
      case DUCKING -> {
        turret.setGoal(Turret.TurretGoal.DUCKING);
      }
    }
  }

  public Command idle() {
    return Commands.startEnd(
            () -> applyState(SuperstructureState.IDLE), () -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Idle");
  }

  public Command intake() {
    return Commands.startEnd(
            () -> applyState(SuperstructureState.INTAKING),
            () -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Intake");
  }

  public Command score() {
    return Commands.sequence(
            Commands.runOnce(() -> applyState(SuperstructureState.SCORING_WINDUP)),
            Commands.waitSeconds(SCORE_WINDUP_SECONDS),
            Commands.waitUntil(activeHubTrigger),
            Commands.runOnce(() -> applyState(SuperstructureState.SCORING)),
            Commands.idle())
        .finallyDo(() -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Score");
  }

  public Command pass() {
    return Commands.sequence(
            Commands.runOnce(() -> applyState(SuperstructureState.PASSING_WINDUP)),
            Commands.waitSeconds(PASS_WINDUP_SECONDS),
            Commands.waitUntil(activeHubTrigger),
            Commands.runOnce(() -> applyState(SuperstructureState.PASSING)),
            Commands.idle())
        .finallyDo(() -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Pass");
  }

  public Command reverse() {
    return Commands.startEnd(
            () -> applyState(SuperstructureState.REVERSING),
            () -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Reverse");
  }

  public Command duck() {
    return Commands.startEnd(
            () -> applyState(SuperstructureState.DUCKING),
            () -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Duck");
  }

  public SuperstructureState getState() {
    return state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/TurretGoal", turret.getGoal());
    Logger.recordOutput("Superstructure/IndexerState", indexer.state);
    Logger.recordOutput("Superstructure/IntakePivot", intake.pivotState);
    Logger.recordOutput("Superstructure/IntakeRoller", intake.rollerState);
  }
}
