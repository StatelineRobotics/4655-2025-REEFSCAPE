// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;

/**
 * Class that returns the pose of the closest scoring position given current position and offset.
 */
public class DriveTarget {

  private static final Translation2d blueReefCenter = new Translation2d(4.489, 4.026);
  private static final Translation2d blueReefCenter = new Translation2d(13.059, 4.026);

  // Pose Keys are driver reletive but pose positions are field reletive
  private static final HashMap<Integer, HashMap<String, Pose2d>> blueReefTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();
  static {
    blueReefTargets.put(0, getScoreingLocations(blueReefCenter, 60, true));
    blueReefTargets.put(1, getScoreingLocations(blueReefCenter, 0, true));
    blueReefTargets.put(2, getScoreingLocations(blueReefCenter, -60, true));
    blueReefTargets.put(3, getScoreingLocations(blueReefCenter, -120, false));
    blueReefTargets.put(4, getScoreingLocations(blueReefCenter, 180, false));
    blueReefTargets.put(5, getScoreingLocations(blueReefCenter, 120, false));
  }
  private static final Pose2d blueProcessorPose = new Pose2d(5.987, 0.451, Rotation2d.fromDegrees(-90));

  private static final HashMap<Integer, HashMap<String, Pose2d>> redReefTargets =
      new HashMap<Integer, HashMap<String, Pose2d>>();
  static {
    redReefTargets.put(0, getScoreingLocations(redReefCenter, -120, true);
    redReefTargets.put(1, getScoreingLocations(redReefCenter, 180, true);
    redReefTargets.put(2, getScoreingLocations(redReefCenter, 120, true);
    redReefTargets.put(3, getScoreingLocations(redReefCenter, 60, false);
    redReefTargets.put(4, getScoreingLocations(redReefCenter, 0, false);
    redReefTargets.put(5, getScoreingLocations(redReefCenter, -60, false);
  }
  private static final Pose2d redProcessorPose = new Pose2d(11.561, 7.601, Rotation2d.fromDegrees(90)); 

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
      blueReefTargets.get(index).get("middle"),
      blueReefTargets.get(index).get("left"),
      blueReefTargets.get(index).get("right")
    };
    return array;
  }

  public static Pose2d[] getRedPoseArray(Integer index) {
    Pose2d[] array = {
      redReefTargets.get(index).get("middle"),
      redReefTargets.get(index).get("left"),
      redReefTargets.get(index).get("right")
    };
    return array;
  }

  private static double getSquaredDistance(Pose2d pose1, Pose2d pose2) {
    return (Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  }

  private static HashMap<String, Pose2d> getClosestSide(Pose2d currentPose, boolean isBlue) {
    Pose2d[] tagPoses;
    if (isBlue) {
      tagPoses = blueTagPoses;
    } else {
      tagPoses = redTagPoses;
    }
    int closestIndex = 0;
    double closestDistance = getSquaredDistance(currentPose, tagPoses[0]);
    System.out.println("WHHYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
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
      return blueReefTargets.get(closestIndex);
    } else {
      return redTargets.get(closestIndex);
    }
  }

  /**
   * @param currentPose
   * @param offset must be middle, right, or left
   * @return closest reef scoring node matching the offset
   */
  public static Pose2d getTargetReefPose(Pose2d currentPose, String offset) {
    System.out.println(currentPose);
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    HashMap<String, Pose2d> side = getClosestSide(currentPose, isBlue);
    System.out.println(side);
    return side.get(offset);
  }

  public static HashMap<String, Pose2d> getScoreingLocations(
      Translation2d center, double angle, boolean flip) {
    HashMap<String, Pose2d> positions = new HashMap<String, Pose2d>();
    // positions.put("center", new Pose2d(center, Rotation2d.fromDegrees(angle)));
    Translation2d middle = center.minus(new Translation2d(1.283, 0));
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

  public static Pose2d getProcesseorPose() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueProcessorPose;
    } else {
      return redProcessorPose;
    }
}
