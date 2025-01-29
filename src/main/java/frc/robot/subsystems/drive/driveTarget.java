// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;

/** Add your docs here. */
public class driveTarget {

  // Pose Keys are driver reletive but pose positions are field reletive
  private static final HashMap<String, Pose2d> blueTargets = new HashMap<String, Pose2d>();

  static {
    blueTargets.put("front", new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    blueTargets.put("frontLeft", new Pose2d(1, 0, Rotation2d.fromDegrees(0)));
  }

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private static final Pose2d[] blueTagePoses = {
    aprilTagLayout.getTagPose(17).get().toPose2d(), // frontRight
    aprilTagLayout.getTagPose(18).get().toPose2d(), // front
    aprilTagLayout.getTagPose(19).get().toPose2d(), // frontLeft
    aprilTagLayout.getTagPose(20).get().toPose2d(), // backLeft
    aprilTagLayout.getTagPose(21).get().toPose2d(), // back
    aprilTagLayout.getTagPose(22).get().toPose2d(), // backRight
  };
}
