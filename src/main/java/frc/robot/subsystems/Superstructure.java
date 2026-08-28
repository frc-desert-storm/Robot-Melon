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
import frc.robot.subsystems.indexer.Indexer.State;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Turret turret;
  private final Indexer indexer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  @AutoLogOutput private SuperstructureState state = SuperstructureState.IDLE;

  @AutoLogOutput
  public final Trigger activeHubTrigger =
      new Trigger(() -> HubShiftUtil.getShiftedShiftInfo().active());

  @AutoLogOutput public final Trigger underTrenchTrigger;

  public Superstructure(
      Turret turret,
      Indexer indexer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.turret = turret;
    this.indexer = indexer;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    underTrenchTrigger =
        Zones.TRENCH_DUCK_ZONES.willContain(poseSupplier, chassisSpeedsSupplier, DUCK_TIME);

    underTrenchTrigger.and(DriverStation::isTeleop).whileTrue(duck());
  }

  public enum SuperstructureState {
    IDLE,
    WINDUP,
    SHOOTING,
    PASSING_WINDUP,
    PASSING,
    REVERSING,
    DUCKING,
    TESTING,
    TESTING_WINDUP
  }

  public void applyState(SuperstructureState newState) {
    state = newState;
    switch (newState) {
      case IDLE -> {
        turret.setGoal(Turret.TurretGoal.IDLE);
        indexer.setState(Indexer.State.IDLE);
      }
      case WINDUP -> {
        turret.setGoal(Turret.TurretGoal.SHOOTING);
        indexer.setState(Indexer.State.IDLE);
      }
      case SHOOTING -> {
        turret.setGoal(Turret.TurretGoal.SHOOTING);
        indexer.setState(Indexer.State.SHOOTING);
      }
      case TESTING -> {
        turret.setGoal(Turret.TurretGoal.TUNING);
        indexer.setState(State.SHOOTING);
      }
      case TESTING_WINDUP -> {
        turret.setGoal(Turret.TurretGoal.TUNING);
      }
      case REVERSING -> {
        turret.setGoal(Turret.TurretGoal.IDLE);
        indexer.setState(Indexer.State.REVERSE);
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

  public Command shoot() {
    return Commands.sequence(
            Commands.runOnce(() -> applyState(SuperstructureState.WINDUP)),
            Commands.waitSeconds(SCORE_WINDUP_SECONDS),
            Commands.waitUntil(activeHubTrigger),
            Commands.runOnce(() -> applyState(SuperstructureState.SHOOTING)),
            Commands.idle())
        .finallyDo(() -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Score");
  }

  public Command test() {
    return Commands.sequence(
            Commands.runOnce(() -> applyState(SuperstructureState.TESTING_WINDUP)),
            Commands.waitSeconds(SCORE_WINDUP_SECONDS),
            Commands.waitUntil(activeHubTrigger),
            Commands.runOnce(() -> applyState(SuperstructureState.TESTING)),
            Commands.idle())
        .finallyDo(() -> applyState(SuperstructureState.IDLE))
        .withName("Superstructure Score");
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
  }
}
