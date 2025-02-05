// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  // Stores the locations of the April Tags
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  // Declaration of PhotonVision cameras
  public PhotonCamera frontCamera = new PhotonCamera("front");
  public PhotonCamera rightCamera = new PhotonCamera("right");
  public PhotonCamera backCamera = new PhotonCamera("back");
  public PhotonCamera leftCamera = new PhotonCamera("left");
  PhotonPoseEstimator photonPoseEstimator;

  PhotonCamera[] cameras = {
      frontCamera,
      rightCamera,
      backCamera,
      leftCamera
  };

  /* 
  ** Camera orientations relative to robot origin.
  ** Transforms are associated by index with PhotonCamera array (i.e. cameraTransforms[0] -> cameras[0])
  ** TODO: Make a physical mark on Gary for what we consider to be the origin. 
  */
  Transform3d[] cameraTransforms = {
      new Transform3d(0, 0.175, 0.4, new Rotation3d(0, 0, 0)),
      new Transform3d(0.495, 0, 0.4, new Rotation3d(0, 0, 1.5 * Math.PI)),
      new Transform3d(0, -0.155, 0.4, new Rotation3d(0, 0, Math.PI)),
      new Transform3d(0.485, 0, 0.4, new Rotation3d(0, 0, 0.5 * Math.PI))                                  // robot origin
  };

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  public void getBestTarget() {

  }

  public void logCameraData() {
    double frontTargetYaw = 0.0;
    double frontTargetRange = 0.0;
    double rightTargetYaw = 0.0;
    double rightTargetRange = 0.0;

    // Vision target data collection
    boolean targetVisible = false;
    var frontResults = frontCamera.getAllUnreadResults();
    if (!frontResults.isEmpty()) {
      var result = frontResults.get(frontResults.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 6) { // HAS TO BE RIGHT ID
            frontTargetYaw = target.getYaw();
            frontTargetRange = PhotonUtils.calculateDistanceToTargetMeters(.275, .65, Units.degreesToRadians(0),
                Units.degreesToRadians(target.getPitch()));
            targetVisible = true;
          }
        }
      }
    }
    var rightResults = rightCamera.getAllUnreadResults();
    if (!rightResults.isEmpty()) {
      var result = rightResults.get(rightResults.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 6) { // HAS TO BE RIGHT ID
            frontTargetYaw = target.getYaw();
            frontTargetRange = PhotonUtils.calculateDistanceToTargetMeters(.275, .65, Units.degreesToRadians(0),
                Units.degreesToRadians(target.getPitch()));
            targetVisible = true;
          }
        }
      }
    }

    SmartDashboard.putNumber("frontTargetYaw", frontTargetYaw);
    SmartDashboard.putNumber("frontTargetRange", frontTargetRange);
    SmartDashboard.putNumber("rightTargetYaw", rightTargetYaw);
    SmartDashboard.putNumber("rightTargetRange", rightTargetRange);
  }

  public void dumpSingleTagCameraData(PhotonCamera[] cameras, Transform3d[] cameraTransforms) {
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      Transform3d currentCameraTransform = cameraTransforms[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.hasTargets()) {          
          PhotonTrackedTarget bestTarget = result.getBestTarget();
          Pose3d estimatedRobotPose = null;
          String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();
          
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.id", bestTarget.getFiducialId());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.x", bestTarget.getBestCameraToTarget().getMeasureX().in(Meters));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.y", bestTarget.getBestCameraToTarget().getMeasureY().in(Meters));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.z", bestTarget.getBestCameraToTarget().getMeasureZ().in(Meters));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.x", bestTarget.getBestCameraToTarget().getRotation().getMeasureX().in(Degrees));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.y", bestTarget.getBestCameraToTarget().getRotation().getMeasureY().in(Degrees));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.rot.z", bestTarget.getBestCameraToTarget().getRotation().getMeasureZ().in(Degrees));
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.ambiguity", bestTarget.getPoseAmbiguity());

          estimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(), currentCameraTransform);
          
          // Dump information to SmartDashboard. RTE = Robot Transform Estimation. RTE.Rot.X = Robot Transform Estimation's Rotation X. 
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.x", estimatedRobotPose.getX());
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.y", estimatedRobotPose.getY());
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.z", estimatedRobotPose.getZ());
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.x", Units.radiansToDegrees(estimatedRobotPose.getRotation().getX()));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.y", Units.radiansToDegrees(estimatedRobotPose.getRotation().getY()));
          SmartDashboard.putNumber(currentCamera.getName() + ".fr.rte.rot.z", Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ()));
          SmartDashboard.putString(currentCamera.getName() + ".fr.rte.timestamp", timestamp);
        } 
      }
    }
  }

  // Updates PhotonPoseEstimator // Need to integrate camera offsets
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose, PhotonPipelineResult result) {
    // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose); // Not needed except for with CLOSEST_TO_REFERENCE_POSE
    return photonPoseEstimator.update(result);
  }

  public void dumpMultiTagData(PhotonCamera[] cameras, Transform3d[] cameraTransform) {
    for(int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      Transform3d currentCameraTransform = cameraTransforms[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCameraTransform);

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.getMultiTagResult().isPresent()) {
          Pose3d estimatedRobotPose = null;
          String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();
          
          estimatedRobotPose = getEstimatedGlobalPose(estimatedRobotPose, result).get().estimatedPose;  

          // Dump information to SmartDashboard. RTE = Robot Transform Estimation. RTE.Rot.X = Robot Transform Estimation's Rotation X. 
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.x", estimatedRobotPose.getX());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.y", estimatedRobotPose.getY());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.z", estimatedRobotPose.getZ());
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.x", Units.radiansToDegrees(estimatedRobotPose.getRotation().getX()));
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.y", Units.radiansToDegrees(estimatedRobotPose.getRotation().getY()));
          SmartDashboard.putNumber(currentCamera.getName() + ".frmt.rte.rot.z", Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ()));
          SmartDashboard.putString(currentCamera.getName() + ".frmt.rte.timestamp", timestamp);
        } 
      }
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // DON'T DELETE

    // dumpSingleTagCameraData(cameras, cameraTransforms);
    dumpMultiTagData(cameras, cameraTransforms);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
