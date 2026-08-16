// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignalCollection;
import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static StatusSignalCollection signals = new StatusSignalCollection();

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static void registerStatusSignals(Frequency frequency, BaseStatusSignal... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);
    PhoenixUtil.registerStatusSignals(signals);
  }

  public static void registerStatusSignals(BaseStatusSignal... signals) {
    PhoenixUtil.signals.addSignals(signals);
  }

  public static StatusCode refreshAll() {
    return PhoenixUtil.signals.refreshAll();
  }

  public static Angle DistanceToAngle(Distance distance) {
    return Rotations.of(distance.in(Inch) / Math.PI);
  }

  public static Distance AngleToDistance(Angle angle) {
    return Inch.of(angle.in(Rotations) * Math.PI);
  }

  public static LinearVelocity AngularVelocityToLinearVelocity(AngularVelocity angularVelocity) {
    return InchesPerSecond.of(angularVelocity.in(RotationsPerSecond) * Math.PI);
  }
}
