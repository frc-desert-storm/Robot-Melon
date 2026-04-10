// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout;

  static {
    try {
      aprilTagLayout =
          new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath() + "/apriltags.json");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Camera names, must match names configured on coprocessor
  public static String leftCameraName = "leftCamera";
  public static String rightCameraName = "rightCamera";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToLeftCamera =
      new Transform3d(
          Units.inchesToMeters(9.839),
          Units.inchesToMeters(10.574),
          Units.inchesToMeters(5.952),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(13)));
  public static Transform3d robotToRightCamera =
      new Transform3d(
          Units.inchesToMeters(9.839),
          Units.inchesToMeters(-10.574),
          Units.inchesToMeters(5.952),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(-27)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.1;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Left camera
        1.0 // Right camera
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
