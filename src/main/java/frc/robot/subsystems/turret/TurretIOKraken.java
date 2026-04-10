// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

public class TurretIOKraken implements TurretIO {
  private final TalonFX turnMotor;
  private final TalonFX hoodMotor;
  private final TalonFX flywheelMotor;

  private final TalonFXConfiguration turnConfig;
  private final TalonFXConfiguration hoodConfig;
  private final TalonFXConfiguration flywheelConfig;

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<Double> turnSetpoint;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;
  private final StatusSignal<Current> turnSupplyCurrent;

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<Double> hoodSetpoint;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;
  private final StatusSignal<Current> hoodSupplyCurrent;

  private final StatusSignal<AngularVelocity> flywheelSpeed;
  private final StatusSignal<AngularAcceleration> flywheelAccel;
  private final StatusSignal<Double> flywheelSetpointSpeed;
  private final StatusSignal<Double> flywheelSetpointAccel;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Current> flywheelSupplyCurrent;

  private final PositionVoltage turnPositionRequest = new PositionVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0);

  private final NeutralOut neutralOut = new NeutralOut();

  public TurretIOKraken() {
    turnMotor = new TalonFX(TURN_ID, TunerConstants.kCANBus);
    hoodMotor = new TalonFX(HOOD_ID, TunerConstants.kCANBus);
    flywheelMotor = new TalonFX(FLYWHEEL_ID, TunerConstants.kCANBus);

    turnConfig =
        new TalonFXConfiguration()
            .withSlot0(TURN_GAINS)
            .withCurrentLimits(TURN_CURRENT_LIMITS)
            .withMotorOutput(TURN_OUTPUT_CONFIGS)
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(MAX_TURN_ANGLE)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(MIN_TURN_ANGLE))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(TURN_TO_TURRET_RATIO));

    hoodConfig =
        new TalonFXConfiguration()
            .withSlot0(HOOD_GAINS)
            .withCurrentLimits(HOOD_CURRENT_LIMITS)
            .withMotorOutput(HOOD_OUTPUT_CONFIGS)
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(MAX_HOOD_ANGLE)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(MIN_HOOD_ANGLE))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_MOTOR_RATIO));
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig =
        new TalonFXConfiguration()
            .withSlot0(FLYWHEEL_GAINS)
            .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
            .withMotorOutput(FLYWHEEL_OUTPUT_CONFIGS)
            .withFeedback(FLYWHEEL_FEEDBACK_CONFIGS);

    PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));

    turnPosition = turnMotor.getPosition();
    turnSetpoint = turnMotor.getClosedLoopReference();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();
    turnSupplyCurrent = turnMotor.getSupplyCurrent();

    hoodPosition = hoodMotor.getPosition();
    hoodSetpoint = hoodMotor.getClosedLoopReference();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getStatorCurrent();
    hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

    flywheelSpeed = flywheelMotor.getVelocity();
    flywheelAccel = flywheelMotor.getAcceleration();
    flywheelSetpointSpeed = flywheelMotor.getClosedLoopReference();
    flywheelSetpointAccel = flywheelMotor.getClosedLoopReferenceSlope();
    flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
    flywheelCurrent = flywheelMotor.getTorqueCurrent();
    flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();

    PhoenixUtil.registerStatusSignals(
        Hertz.of(50),
        turnPosition,
        turnSetpoint,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent,
        turnSupplyCurrent,
        hoodPosition,
        hoodSetpoint,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent,
        hoodSupplyCurrent,
        flywheelSpeed,
        flywheelAccel,
        flywheelSetpointSpeed,
        flywheelSetpointAccel,
        flywheelAppliedVolts,
        flywheelCurrent,
        flywheelSupplyCurrent);
    turnMotor.optimizeBusUtilization();
    hoodMotor.optimizeBusUtilization();
    flywheelMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turnMotorConnected =
        BaseStatusSignal.isAllGood(
            turnPosition,
            turnSetpoint,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            turnSupplyCurrent);
    inputs.turnPosition = turnPosition.getValue();
    inputs.turnSetpoint = Rotations.of(turnSetpoint.getValueAsDouble());
    inputs.turnVelocity = turnVelocity.getValue();
    inputs.turnAppliedVolts = turnAppliedVolts.getValue();
    inputs.turnCurrent = turnCurrent.getValue();
    inputs.turnSupplyCurrent = turnSupplyCurrent.getValue();

    inputs.hoodMotorConnected =
        BaseStatusSignal.isAllGood(
            hoodPosition,
            hoodSetpoint,
            hoodVelocity,
            hoodAppliedVolts,
            hoodCurrent,
            hoodSupplyCurrent);
    inputs.hoodPosition = hoodPosition.getValue();
    inputs.hoodSetpoint = Rotations.of(hoodSetpoint.getValueAsDouble());
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValue();
    inputs.hoodCurrent = hoodCurrent.getValue();
    inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValue();

    inputs.flywheelMotorConnected =
        BaseStatusSignal.isAllGood(
            flywheelSpeed,
            flywheelAccel,
            flywheelSetpointSpeed,
            flywheelSetpointAccel,
            flywheelAppliedVolts,
            flywheelCurrent,
            flywheelSupplyCurrent);
    inputs.flywheelSpeed = flywheelSpeed.getValue();
    inputs.flywheelAccel = flywheelAccel.getValue();
    inputs.flywheelSetpointSpeed = RotationsPerSecond.of(flywheelSetpointSpeed.getValueAsDouble());
    inputs.flywheelSetpointAccel =
        RotationsPerSecondPerSecond.of(flywheelSetpointAccel.getValueAsDouble());
    inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
    inputs.flywheelCurrent = flywheelCurrent.getValue();
    inputs.flywheelSupplyCurrent = flywheelCurrent.getValue();
  }

  @Override
  public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
    turnMotor.setControl(turnPositionRequest.withPosition(position).withVelocity(velocity));
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodMotor.setControl(hoodPositionRequest.withPosition(angle));
  }

  @Override
  public void setHoodOut(Voltage out) {
    hoodMotor.setControl(hoodVoltageRequest.withOutput(out));
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(speed));
  }

  @Override
  public void stopTurn() {
    turnMotor.setControl(neutralOut);
  }

  @Override
  public void stopHood() {
    hoodMotor.setControl(neutralOut);
  }

  @Override
  public void stopFlywheel() {
    flywheelMotor.setControl(neutralOut);
  }

  @Override
  public void resetTurnEncoder() {
    turnMotor.setPosition(turnPosition.getValue().in(Rotations) % 1);
  }

  @Override
  public void zeroHoodPosition() {
    hoodMotor.setPosition(MIN_HOOD_ANGLE);
  }

  @Override
  public void setTurnPID(double kP, double kD, double kV, double kS) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kD = kD;
    turnConfig.Slot0.kV = kV;
    turnConfig.Slot0.kS = kS;

    PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig.Slot0, 0.25));
  }

  @Override
  public void setHoodPID(double kP, double kD, double kS) {
    hoodConfig.Slot0.kP = kP;
    hoodConfig.Slot0.kD = kD;
    hoodConfig.Slot0.kS = kS;

    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig.Slot0, 0.25));
  }

  @Override
  public void setFlywheelPID(double kP, double kD, double kV, double kS) {
    flywheelConfig.Slot0.kP = kP;
    flywheelConfig.Slot0.kD = kD;
    flywheelConfig.Slot0.kV = kV;
    flywheelConfig.Slot0.kS = kS;

    PhoenixUtil.tryUntilOk(
        5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig.Slot0, 0.25));
  }
}
