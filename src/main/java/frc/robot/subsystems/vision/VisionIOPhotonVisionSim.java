// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;
  private static double lastUpdateSec = -1.0;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Transform3d> renderCameraSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this(name, () -> robotToCamera, poseSupplier);
  }

  public VisionIOPhotonVisionSim(
      String name, Supplier<Transform3d> robotToCameraSupplier, Supplier<Pose2d> poseSupplier) {
    this(name, timestamp -> robotToCameraSupplier.get(), robotToCameraSupplier, poseSupplier);
  }

  /**
   * Creates a new VisionIOPhotonVisionSim with separate render and decode transforms.
   *
   * @param name The name of the camera.
   * @param robotToCameraFn Timestamp-aware transform used to decode results (matches real code,
   *     e.g. {@code timestamp -> getRobotToTurretCamera(turret.getTurnPositionAt(timestamp))}).
   * @param renderCameraSupplier Ground-truth current transform used to render the sim image.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      DoubleFunction<Transform3d> robotToCameraFn,
      Supplier<Transform3d> renderCameraSupplier,
      Supplier<Pose2d> poseSupplier) {
    super(name, robotToCameraFn);
    this.poseSupplier = poseSupplier;
    this.renderCameraSupplier = renderCameraSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, renderCameraSupplier.get());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Render with ground-truth current camera pose.
    visionSim.adjustCamera(cameraSim, renderCameraSupplier.get());
    // Step the sim once per loop (not once per camera) so each camera doesn't
    // push duplicate results. Other cameras render with last tick's pose (1-tick / ~20ms delay).
    double now = Timer.getFPGATimestamp();
    if (now != lastUpdateSec) {
      lastUpdateSec = now;
      visionSim.update(poseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
