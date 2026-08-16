package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.INTAKING_POSE;
import static frc.robot.Constants.IntakeConstants.STOW_POSE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
    Logger.recordOutput("Intake/extensionState", extensionState);
    Logger.recordOutput("Intake/rollerState", rollerState);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/extensionState", extensionState);
    Logger.recordOutput("Intake/rollerState", rollerState);
    switch (extensionState) {
      case EXTENDED -> {
        if (rollerState == RollerState.INTAKING) {
          io.setRollerSpeed(RotationsPerSecond.of(1000.0 / 60));
        }
        if (INTAKING_POSE.in(Inch) - inputs.extensionLeftPosition.in(Inch) > 1.5
            | INTAKING_POSE.in(Inch) - inputs.extensionPosition.in(Inch) > 1.5) {
          setState(ExtensionState.RETRACTING, rollerState);
        }
      }
      case EXTENDING -> {
        if (Math.abs(inputs.extensionLeftPosition.in(Inch) - INTAKING_POSE.in(Inch)) < 0.25
            && Math.abs(inputs.extensionPosition.in(Inch) - INTAKING_POSE.in(Inch)) < 0.25) {
          io.stopExtension();
          extensionState = ExtensionState.EXTENDED;
        }
      }
      case RETRACTING -> {}
    }
  }

  public void setState(ExtensionState extensionState, RollerState rollerState) {
    this.extensionState = extensionState;
    this.rollerState = rollerState;
    switch (extensionState) {
      case IDLE -> io.stopExtension();
      case RETRACTING -> io.setExtensionDistance(STOW_POSE);
      case EXTENDING -> io.setExtensionDistance(INTAKING_POSE);
    }
    switch (rollerState) {
        // case INTAKING -> io.setRollerSpeed(RotationsPerSecond.of(1000.0 / 60));
      case REVERSE -> io.setRollerSpeed(RotationsPerSecond.of(-1000.0 / 60));
      case IDLE -> io.stopRoller();
    }
  }

  public void stop() {
    this.rollerState = RollerState.IDLE;
    this.extensionState = ExtensionState.IDLE;
    io.stopRoller();
    io.stopExtension();
  }

  public Command intake() {
    return Commands.startEnd(
        () -> setState(ExtensionState.EXTENDING, RollerState.INTAKING),
        () -> setState(ExtensionState.RETRACTING, RollerState.IDLE),
        this);
  }

  public ExtensionState extensionState = ExtensionState.IDLE;

  public enum ExtensionState {
    EXTENDED,
    EXTENDING,
    RETRACTED,
    RETRACTING,
    IDLE
  }

  public RollerState rollerState = RollerState.IDLE;

  public enum RollerState {
    INTAKING,
    REVERSE,
    IDLE
  }
}
