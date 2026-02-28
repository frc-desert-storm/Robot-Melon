// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final String CANIVORE_BUS = "CANivore";

  public static final double turretRPMRight = 50;
  public static final double turretRPMLeft = -50;
  public static final double turretRightHardstopAngle = 120;
  public static final double turretLeftHardstopAngle = -120;
  public static final double turretAngleTolerance = 25;

  public static double shooterForwardRPM = 2000;
  public static double shooterBackwardRPM = -2000;

  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int LIFT_CAN_ID = 20;
    public static final int ROLLER_CAN_ID = 21;
    public static final int CONVEYOR_CAN_ID = 22;
  }

  public static final class IndexerConstants {
    private IndexerConstants() {}

    public static final int LEADER_CAN_ID = 30;
    public static final int FOLLOWER_CAN_ID = 31;
  }

  public static final int OPERATOR_KEYBOARD_PORT = 2;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
