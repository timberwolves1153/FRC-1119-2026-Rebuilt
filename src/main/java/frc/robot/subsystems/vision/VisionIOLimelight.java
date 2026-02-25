// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleSubscriber latencySubscriber;
  private final NetworkTable telemetryTable;
  private final StructPublisher<Pose2d> posePublisher;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    this.rotationSupplier = rotationSupplier;

    var table = NetworkTableInstance.getDefault().getTable(name);
    this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
    this.posePublisher =
        telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    LimelightHelpers.SetRobotOrientation(
        name, this.rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    final PoseEstimate poseEstimate_MegaTag2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (poseEstimate_MegaTag1 == null
        || poseEstimate_MegaTag2 == null
        || poseEstimate_MegaTag1.tagCount == 0
        || poseEstimate_MegaTag2.tagCount == 0) {
      return;
    }

    // Combine the readings from MegaTag1 and MegaTag2:
    // 1. Use the more stable position from MegaTag2
    // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
    poseEstimate_MegaTag1.pose =
        new Pose2d(
            poseEstimate_MegaTag1.pose.getTranslation(), poseEstimate_MegaTag1.pose.getRotation());
    poseEstimate_MegaTag2.pose =
        new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(), poseEstimate_MegaTag1.pose.getRotation());

    inputs.poseObservations[0] =
        new Measurement(poseEstimate_MegaTag1.pose, poseEstimate_MegaTag1.timestampSeconds);
    inputs.poseObservations[1] =
        new Measurement(poseEstimate_MegaTag2.pose, poseEstimate_MegaTag2.timestampSeconds);
    // (poseEstimate_MegaTag1.pose, poseEstimate_MegaTag2.pose);

    posePublisher.set(poseEstimate_MegaTag2.pose);
  }
}
