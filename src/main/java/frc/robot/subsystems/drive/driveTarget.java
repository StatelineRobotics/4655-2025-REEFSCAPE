// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;

/** Class that returns the pose of the closest scoring position given current position and offset. */
public class driveTarget {

  // Pose Keys are driver reletive but pose positions are field reletive
  private static final HashMap<String, Pose2d> blueTargets = new HashMap<String, Pose2d>();
  static {
    blueTargets.put("front", new Pose2d(4.026, 3.207, Rotation2d.fromDegrees(0)));
    blueTargets.put("frontL", new Pose2d(4.190, 3.207, Rotation2d.fromDegrees(0)));
    blueTargets.put("frontR", new Pose2d(3.862, 3.207, Rotation2d.fromDegrees(0)));
    blueTargets.put("frontRight", new Pose2d(2.834, 3.991, Rotation2d.fromDegrees(-60)));
    blueTargets.put("frontRightL", new Pose2d(2.998, 3.706, Rotation2d.fromDegrees(-60)));
    blueTargets.put("frontRightR", new Pose2d(2.916, 3.849, Rotation2d.fromDegrees(-60)));
    blueTargets.put("frontLeft", new Pose2d(5.218, 3.991, Rotation2d.fromDegrees(30)));
    blueTargets.put("frontLeftL", new Pose2d(5.136, 3.848, Rotation2d.fromDegrees(30)));
    blueTargets.put("frontLeftR", new Pose2d(5.055, 3.706, Rotation2d.fromDegrees(30)));
  }
  private static final hashMap<String, Pose2d> redTargets = new HashMap<String, Pose2d();
  static {

  }

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
    aprilTagLayout.getTagPose(9).get().toPose2d(), // backRight
  }

  private static double getSquaredDistance(Pose2d pose1, Pose2d pose2) {
    return (Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  }

  private int getClosestSide(Pose2d currentPose, boolean isRed) {
    Pose[] tagPoses;
    if (isRed) {
      tagPoses = redTagPoses;
    } else {
      tagPoses = blueTagPoses;
    }
    int closestIndex = 0;
    double closestDistance = getSquaredDistance(currentPose, blueTagPoses[0]);
    for (int i = 1; i < blueTagPoses.length; i++) {
        double distance = getSquaredDistance(currentPose, blueTagPoses[i]);
        if (distance < closestDistance) {
            closestIndex = i;
            closestDistance = distance;
        }
    }
    return closestIndex;
  }

  public Pose2d getTargetReefPose(Pose2d currentPose, String offset) {
      if (color == blue) {
        targets = blueTargets;
      } else {
        targets = redTargets;
      int targetSideIndex = getClosestSide(currentPose);
      switch (expression) {
          
          case 0:
              if (offset.equals("left")) {
                return targets.get(frontRightL);
              } else if (offset.equals("right")) {
                return targets.get(frontRightR);
              } else {
                return targets.get(frontRight);
              }
              break;

          case 1:
              if (offset.equals("left")) {
                return targets.get(frontL);
              } else if (offset.equals("right")) {
                return targets.get(frontR);
              } else {
                return targets.get(front);
              }
              break;

          case 2:
              if (offset.equals("left")) {
                return targets.get(frontLeftL);
              } else if (offset.equals("right")) {
                return targets.get(frontLeftR);
              } else {
                return targets.get(frontLeft);
              }
              break;

        case 3:
          if (offset.equals("left")) {
            return targets.get(backLeftL);
          } else if (offset.equals("right")) {
            return targets.get(backLeftR);
          } else {
            return targets.get(backLeft);
          }
          break;

        case 4: 
          if (offset.equals("left")) {
            return targets.get(backL);
          } else if (offset.equals("right")) {
            return targets.get(backR);
          } else {
            return targets.get(back);
          }
          break;

        case 5:
          if (offset.equals("left")) {
            return targets.get(backRightL);
          } else if (offset.equals("right")) {
            return targets.get(backR);
          } else {
            return targets.get(backRight):
          }
          break;
  
}
