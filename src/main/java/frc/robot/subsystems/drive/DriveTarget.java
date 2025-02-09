// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;

/**
 * Class that returns the pose of the closest scoring position given current position and offset.
 */
public class DriveTarget {
  private static final double reefOffset = 1.283;
  private static final double startOffset = 0.25;

  private static final Translation2d blueReefCenter = new Translation2d(4.489, 4.026);

  // Pose Keys are driver reletive but pose positions are field reletive
  private static final HashMap<Integer, HashMap<String, Pose2d>> blueReefEndTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();

  static {
    blueReefEndTargets.put(0, getScoreingLocations(blueReefCenter, reefOffset, 60, true));
    blueReefEndTargets.put(1, getScoreingLocations(blueReefCenter, reefOffset, 0, true));
    blueReefEndTargets.put(2, getScoreingLocations(blueReefCenter, reefOffset, -60, true));
    blueReefEndTargets.put(3, getScoreingLocations(blueReefCenter, reefOffset, -120, false));
    blueReefEndTargets.put(4, getScoreingLocations(blueReefCenter, reefOffset, 180, false));
    blueReefEndTargets.put(5, getScoreingLocations(blueReefCenter, reefOffset, 120, false));
  }

  private static final HashMap<Integer, HashMap<String, Pose2d>> blueReefStartTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();

  static {
    blueReefStartTargets.put(
        0, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 60, true));
    blueReefStartTargets.put(
        1, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 0, true));
    blueReefStartTargets.put(
        2, getScoreingLocations(blueReefCenter, reefOffset + startOffset, -60, true));
    blueReefStartTargets.put(
        3, getScoreingLocations(blueReefCenter, reefOffset + startOffset, -120, false));
    blueReefStartTargets.put(
        4, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 180, false));
    blueReefStartTargets.put(
        5, getScoreingLocations(blueReefCenter, reefOffset + startOffset, 120, false));
  }

  private static final Pose2d blueProcessorEndPose =
      new Pose2d(5.987, 0.451, Rotation2d.fromDegrees(-90));
  private static final Pose2d blueProcessorStartPose =
      blueProcessorEndPose.transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kZero));
  private static final Pose2d[] blueSourceEndPoses = {
    new Pose2d(1.111, 1.013, Rotation2d.fromDegrees(-126 + 180)),
    new Pose2d(1.111, 7.038, Rotation2d.fromDegrees(126 - 180))
  };

  private static final Pose2d[] blueSourceStartPoses = {
    new Pose2d(1.405, 1.418, Rotation2d.fromDegrees(-126 + 180)),
    new Pose2d(1.405, 6.634, Rotation2d.fromDegrees(126 - 180))
  };

  private static final Translation2d redReefCenter = new Translation2d(13.059, 4.026);
  private static final HashMap<Integer, HashMap<String, Pose2d>> redReefEndTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();

  static {
    redReefEndTargets.put(0, getScoreingLocations(redReefCenter, reefOffset, -120, true));
    redReefEndTargets.put(1, getScoreingLocations(redReefCenter, reefOffset, 180, true));
    redReefEndTargets.put(2, getScoreingLocations(redReefCenter, reefOffset, 120, true));
    redReefEndTargets.put(3, getScoreingLocations(redReefCenter, reefOffset, 60, false));
    redReefEndTargets.put(4, getScoreingLocations(redReefCenter, reefOffset, 0, false));
    redReefEndTargets.put(5, getScoreingLocations(redReefCenter, reefOffset, -60, false));
  }

  private static final HashMap<Integer, HashMap<String, Pose2d>> redReefStartTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();

  static {
    redReefStartTargets.put(
        0, getScoreingLocations(redReefCenter, startOffset + reefOffset, -120, true));
    redReefStartTargets.put(
        1, getScoreingLocations(redReefCenter, startOffset + reefOffset, 180, true));
    redReefStartTargets.put(
        2, getScoreingLocations(redReefCenter, startOffset + reefOffset, 120, true));
    redReefStartTargets.put(
        3, getScoreingLocations(redReefCenter, startOffset + reefOffset, 60, false));
    redReefStartTargets.put(
        4, getScoreingLocations(redReefCenter, startOffset + reefOffset, 0, false));
    redReefStartTargets.put(
        5, getScoreingLocations(redReefCenter, startOffset + reefOffset, -60, false));
  }

  private static final Pose2d[] redSourceEndPoses = {
    new Pose2d(16.437, 7.038, Rotation2d.fromDegrees(126)),
    new Pose2d(16.437, 1.013, Rotation2d.fromDegrees(-126))
  };

  private static final Pose2d[] redSourceStartPoses = {
    new Pose2d(16.143, 6.634, Rotation2d.fromDegrees(126)),
    new Pose2d(16.143, 1.418, Rotation2d.fromDegrees(-126))
  };

  private static final Pose2d redProcessorEndPose =
      new Pose2d(11.561, 7.601, Rotation2d.fromDegrees(90));
  private static final Pose2d redProcessorStartPose =
      redProcessorEndPose.transformBy(new Transform2d(0.5, 0.0, Rotation2d.kZero));

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private static final Pose2d[] blueTagPoses = {
    aprilTagLayout.getTagPose(17).get().toPose2d(), // frontRight
    aprilTagLayout.getTagPose(18).get().toPose2d(), // front
    aprilTagLayout.getTagPose(19).get().toPose2d(), // frontLeft
    aprilTagLayout.getTagPose(20).get().toPose2d(), // backLeft
    aprilTagLayout.getTagPose(21).get().toPose2d(), // back
    aprilTagLayout.getTagPose(22).get().toPose2d(), // backRight
  };
  private static final Pose2d[] redTagPoses = {
    aprilTagLayout.getTagPose(8).get().toPose2d(), // frontRight
    aprilTagLayout.getTagPose(7).get().toPose2d(), // front
    aprilTagLayout.getTagPose(6).get().toPose2d(), // frontLeft
    aprilTagLayout.getTagPose(11).get().toPose2d(), // backLeft
    aprilTagLayout.getTagPose(10).get().toPose2d(), // back
    aprilTagLayout.getTagPose(9).get().toPose2d() // backRight
  };

  public static Pose2d[] getBluePoseArray(Integer index) {
    Pose2d[] array = {
      blueReefEndTargets.get(index).get("middle"),
      blueReefEndTargets.get(index).get("left"),
      blueReefEndTargets.get(index).get("right")
    };
    return array;
  }

  private static double getSquaredDistance(Pose2d pose1, Pose2d pose2) {
    return (Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  }

  private static Pose2d[] getClosestPath(Pose2d currentPose, boolean isBlue, String offset) {
    Pose2d[] tagPoses;
    if (isBlue) {
      tagPoses = blueTagPoses;
    } else {
      tagPoses = redTagPoses;
    }
    int closestIndex = 0;
    double closestDistance = getSquaredDistance(currentPose, tagPoses[0]);
    for (int i = 1; i < tagPoses.length; i++) {
      double distance = getSquaredDistance(currentPose, tagPoses[i]);
      System.out.println(currentPose);
      System.out.println(tagPoses[i]);
      System.out.println(distance);
      if (distance < closestDistance) {
        closestIndex = i;
        closestDistance = distance;
      }
    }
    if (isBlue) {
      Pose2d[] targets = {
        blueReefStartTargets.get(closestIndex).get(offset),
        blueReefEndTargets.get(closestIndex).get(offset)
      };
      return targets;
    } else {
      Pose2d[] targets = {
        redReefStartTargets.get(closestIndex).get(offset),
        redReefEndTargets.get(closestIndex).get(offset)
      };
      return targets;
    }
  }

  public static Pose2d[] getTargetReefPose(Pose2d currentPose, String offset) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Pose2d[] poses = getClosestPath(currentPose, isBlue, offset);
    return poses;
  }

  public static HashMap<String, Pose2d> getScoreingLocations(
      Translation2d center, double offset, double angle, boolean flip) {
    HashMap<String, Pose2d> positions = new HashMap<String, Pose2d>();
    // positions.put("center", new Pose2d(center, Rotation2d.fromDegrees(angle)));
    Translation2d middle = center.minus(new Translation2d(offset, 0));
    Translation2d left;
    Translation2d right;
    if (flip) {
      left = middle.plus(new Translation2d(0, 0.1651));
      right = middle.minus(new Translation2d(0, 0.1651));
    } else {
      left = middle.minus(new Translation2d(0, 0.1651));
      right = middle.plus(new Translation2d(0, 0.1651));
    }
    middle = middle.rotateAround(center, Rotation2d.fromDegrees(angle));
    left = left.rotateAround(center, Rotation2d.fromDegrees(angle));
    right = right.rotateAround(center, Rotation2d.fromDegrees(angle));
    positions.put("middle", new Pose2d(middle, Rotation2d.fromDegrees(angle)));
    positions.put("left", new Pose2d(left, Rotation2d.fromDegrees(angle)));
    positions.put("right", new Pose2d(right, Rotation2d.fromDegrees(angle)));
    return positions;
  }

  public static Pose2d[] getProcesseorPose() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      Pose2d[] poses = {blueProcessorStartPose, blueProcessorEndPose};
      return poses;
    } else {
      Pose2d[] poses = {redProcessorStartPose, redProcessorEndPose};
      return poses;
    }
  }

  public static Pose2d[] getSourcePose(Pose2d currentPose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (getSquaredDistance(currentPose, blueSourceEndPoses[0])
          < getSquaredDistance(currentPose, blueSourceEndPoses[1])) {
        Pose2d[] poses = {blueSourceStartPoses[0], blueSourceEndPoses[0]};
        return poses;
      } else {
        Pose2d[] poses = {blueSourceStartPoses[1], blueSourceEndPoses[1]};
        return poses;
      }
    } else {
      if (getSquaredDistance(currentPose, redSourceEndPoses[0])
          < getSquaredDistance(currentPose, redSourceEndPoses[1])) {
        Pose2d[] poses = {redSourceStartPoses[0], redSourceEndPoses[0]};
        return poses;
      } else {
        Pose2d[] poses = {redSourceStartPoses[1], redSourceEndPoses[1]};
        return poses;
      }
    }
  }
}
