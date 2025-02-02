// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;
import java.util.List;

/**
 * Class that returns the pose of the closest scoring position given current position and offset.
 */
public class DriveTarget {
  private static final PathConstraints constraints =
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
  private static final double reefOffset = 1.283;
  private static final double startOffset = 0.75;

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
    new Pose2d(1.111, 1.013, Rotation2d.fromDegrees(-126)),
    new Pose2d(1.111, 7.038, Rotation2d.fromDegrees(126))
  };

  private static final Pose2d[] blueSourceStartPoses = {
    new Pose2d(1.405, 1.418, Rotation2d.fromDegrees(126)),
    new Pose2d(1.405, 6.634, Rotation2d.fromDegrees(-126))
  };

  private static Pose2d[] redSourceEndPoses = {
    new Pose2d(16.437, 7.038, Rotation2d.fromDegrees(126)),
    new Pose2d(16.437, 1.013, Rotation2d.fromDegrees(-126))
  };

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
    Pose2d[] targets = {
      blueReefStartTargets.get(closestIndex).get(offset),
      blueReefEndTargets.get(closestIndex).get(offset)
    };
    return targets;
  }

  /**
   * @param currentPose
   * @param offset must be middle, right, or left
   * @return closest reef scoring node matching the offset
   */
  public static PathPlannerPath getTargetReefPath(Pose2d currentPose, String offset) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Pose2d[] poses = getClosestPath(currentPose, isBlue, offset);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses[0], poses[1]);
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                poses[1].getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = false;
    return path;
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

  public static PathPlannerPath getProcesseorPose() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(blueProcessorStartPose, blueProcessorEndPose);
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                blueProcessorEndPose
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = false;
    return path;
  }

  public static PathPlannerPath getSourcePose(Pose2d currentPose) {
    int index = 0;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (getSquaredDistance(currentPose, blueSourceEndPoses[0])
          < getSquaredDistance(currentPose, blueSourceEndPoses[1])) {
        index = 0;
      } else {
        index = 1;
      }
    } else {
      if (getSquaredDistance(currentPose, redSourceEndPoses[0])
          < getSquaredDistance(currentPose, redSourceEndPoses[1])) {
        index = 0;
      } else {
        index = 1;
      }
    }

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(blueSourceStartPoses[index], blueSourceEndPoses[index]);
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use
    // unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                blueSourceEndPoses[index]
                    .getRotation()) // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = false;
    return path;
  }
}
