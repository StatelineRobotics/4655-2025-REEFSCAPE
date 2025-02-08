package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera Left = new PhotonCamera("Left");
  private PhotonCamera Right = new PhotonCamera("Right");
  private PhotonCamera Back = new PhotonCamera("Back");
  private PhotonPipelineResult latestResult;
  private VisionLEDMode ledMode = VisionLEDMode.kOff;
  private final PhotonVisionIO io;
  private final PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();
  // private PhotonVisionPoseEstimation poseEstimation = new PhotonVisionPoseEstimation();

  // NJ  Util.consoleLog("PhotonVision created!");

  public PhotonVision(PhotonVisionIO io) {
    this.io = io;
  }

  public Optional<Pose2d> getEstimatedPose() {

    Pose2d averagePose = null;
    Double sumX = 0.0;
    Double sumY = 0.0;
    double poses = 0.0;
    Rotation2d sumRotation = new Rotation2d();
    // double sumConfidence = 0;
    /*
    Pose2d LeftPose = inputs.estimatedLeftPose.toPose2d();
    SmartDashboard.putNumber("VisionPoseLeftX",LeftPose.getX());
    SmartDashboard.putNumber("VisionPoseLeftY",LeftPose.getY());
    if (Left.getLatestResult().hasTargets()) {
        sumX += LeftPose.getX();
        sumY += LeftPose.getY();
        sumRotation = sumRotation.plus(LeftPose.getRotation());
        poses +=1;
    }
    Pose2d RightPose = inputs.estimatedRightPose.toPose2d();
    if (Right.getLatestResult().hasTargets()) {
        sumX += RightPose.getX();
        sumY += RightPose.getY();
        sumRotation = sumRotation.plus(RightPose.getRotation());
        poses +=1;
    }
    */
    Pose2d BackPose = inputs.estimatedBackPose.toPose2d();
    if (Back.getLatestResult().hasTargets()) {
      sumX += BackPose.getX();
      sumY += BackPose.getY();
      sumRotation = sumRotation.plus(BackPose.getRotation());
      poses += 1;
    }
    SmartDashboard.putNumber("VisionPoseBack", sumX);
    SmartDashboard.putNumber("VisionPoseBack", sumY);

    if (poses > 0) {
      sumX = sumX / poses;
      sumY = sumY / poses;
      poses = 0;
    }

    averagePose = new Pose2d(sumX, sumY, sumRotation);

    return Optional.ofNullable(averagePose);
  }

  /**
   * Get the lastest target results object returned by the camera.
   *
   * @return Results object.
   */
  public PhotonPipelineResult getLatestResult() {
    latestResult = Back.getLatestResult();
    return latestResult;
  }

  public double getTimestamp() {
    // return inputs.estimatedRightPoseTimestamp > inputs.estimatedLeftPoseTimestamp ?
    // inputs.estimatedRightPoseTimestamp : inputs.estimatedLeftPoseTimestamp;
    return inputs.estimatedBackPoseTimestamp;
  }
  /**
   * Indicates if lastest camera results list contains targets. Must call getLatestResult() before
   * calling.
   *
   * @return True if targets available, false if not.
   */
  public boolean hasTargets() {
    // getLatestResult();

    return Back.getLatestResult().hasTargets();
  }

  /**
   * Returns the target with the given Fiducial ID
   *
   * @param id the desired Fiducial ID
   * @return the target or null if the ID is not currently being tracked
   */
  public PhotonTrackedTarget getTarget(int id) {
    if (hasTargets()) {
      List<PhotonTrackedTarget> targets = latestResult.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        PhotonTrackedTarget target = targets.get(i);
        if (target.getFiducialId() == id) return target;
      }
      return null;
    } else return null;
  }

  /**
   * Get an array of the currently tracked Fiducial IDs
   *
   * @return an ArrayList of the tracked IDs
   */
  public ArrayList<Integer> getTrackedIDs() {
    ArrayList<Integer> ids = new ArrayList<Integer>();
    if (hasTargets()) {
      List<PhotonTrackedTarget> targets = latestResult.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        ids.add(targets.get(i).getFiducialId());
      }
    }
    return ids;
  }

  /**
   * Checks whether or not the camera currently sees a target with the given Fiducial ID
   *
   * @param id the Fiducial ID
   * @return whether the camera sees the ID
   */
  public boolean hasTarget(int id) {
    return getTrackedIDs().contains(id);
  }

  public boolean getPoseAmbiguity() {
    boolean Updateokay = true;
    // if(inputs.LeftAmbiguitySum >= .6 || inputs.RightAmbiguitySum >= .6){
    if (inputs.BackAmbiguitySum >= .6) {
      Updateokay = false;
    }
    return Updateokay;
  }

  // Best Target Methods =============================================================

  /**
   * Returns the yaw angle of the best target in the latest camera results list. Must call
   * hasTargets() before calling this function.
   *
   * @return Best target yaw value from straight ahead or zero. -yaw means target is left of robot
   *     center.
   */
  public double getYaw() {
    if (hasTargets()) return latestResult.getBestTarget().getYaw();
    else return 0;
  }

  /**
   * Returns the Fiducial ID of the current best target, you should call hasTargets() first!
   *
   * @return the ID or -1 if no targets
   */
  public int getFiducialID() {
    if (hasTargets()) return latestResult.getBestTarget().getFiducialId();
    else return -1;
  }

  /**
   * Returns the area of the best target in the latest camera results list. Must call hasTargets()
   * before calling this function.
   *
   * @return Best target area value.
   */
  public double getArea() {
    if (hasTargets()) return latestResult.getBestTarget().getArea();
    else return 0;
  }

  // Utility Methods =============================================================

  /**
   * Select camera's image processing pipeline.
   *
   * @param index Zero based number of desired pipeline.
   */
  public void selectPipeline(int index) {

    Back.setPipelineIndex(index);
  }

  /**
   * Set the LED mode.
   *
   * @param mode Desired LED mode.
   */
  public void setLedMode(VisionLEDMode mode) {
    // NJ      Util.consoleLog("%d", mode.value);

    Back.setLED(mode);

    ledMode = mode;
  }

  /** Toggle LED mode on/off. */
  public void toggleLedMode() {
    if (ledMode == VisionLEDMode.kOff) ledMode = VisionLEDMode.kOn;
    else ledMode = VisionLEDMode.kOff;

    setLedMode(ledMode);
  }

  /** Save pre-processed image from camera stream. */
  public void inputSnapshot() {
    // NJ      Util.consoleLog();

    Back.takeInputSnapshot();
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Save post-processed image from camera stream. */
  public void outputSnapshot() {
    // NJ      Util.consoleLog();

    Back.takeOutputSnapshot();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // super.initSendable(builder);
    builder.setSmartDashboardType("Subsystem");

    builder.addBooleanProperty("has Targets", () -> hasTargets(), null);
    builder.addDoubleProperty("target yaw", () -> getYaw(), null);
    builder.addDoubleProperty("target area", () -> getArea(), null);
  }

  /**
   * returns an Optional value of the robot's estimated field-centric pose given current tags that
   * it sees. (and also the timestamp)
   *
   * @return the Optional estimated pose (empty optional means no pose or uncertain/bad pose)
   */
  //   @Override
  //   public void periodic() {
  //     Logger.processInputs("PhotonVision", inputs);
  // }
}
