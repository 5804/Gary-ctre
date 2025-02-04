// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
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
      new Transform3d(0, 0.175, 0.35, new Rotation3d(0, 0, 0)),
      new Transform3d(0.495, 0, 0.35, new Rotation3d(0, 0, 1.5 * Math.PI)),
      new Transform3d(0, -0.155, 0.35, new Rotation3d(0, 0, Math.PI)),
      new Transform3d(0.485, 0, 0.35, new Rotation3d(0, 0, 0.5 * Math.PI))                                  // robot origin
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

  public void dumpCameraData(PhotonCamera[] cameras, Transform3d[] camreaTransforms) {
    for(int cameraIndex=0; cameraIndex<cameras.length; cameraIndex++){
      PhotonCamera currentCamera = cameras[cameraIndex];
      Transform3d currentCameraTransform = camreaTransforms[cameraIndex];
      List<PhotonPipelineResult> frameResults = currentCamera.getAllUnreadResults();

      if (!frameResults.isEmpty()) {
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        if (result.hasTargets()) {          
          PhotonTrackedTarget bestTarget = result.getBestTarget();
          double bestTargetHeight = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getZ();
          double bestTargetPitch = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get().getRotation().getY();
          double currentCameraHeight = currentCameraTransform.getZ();
          double currentCameraPitch = currentCameraTransform.getRotation().getY();
          double bestTargetRange = PhotonUtils.calculateDistanceToTargetMeters(currentCameraHeight, bestTargetHeight, currentCameraPitch, bestTargetPitch);
          Pose3d estimatedRobotPose = null;
          String timestamp = "" + java.time.LocalDateTime.now().getHour() + ":" + java.time.LocalDateTime.now().getMinute() + ":" + java.time.LocalDateTime.now().getSecond();
          
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.id", bestTarget.getFiducialId());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.p", bestTarget.getPitch());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.y", bestTarget.getYaw());
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.r", bestTargetRange);
          SmartDashboard.putNumber(currentCamera.getName() + ".bt.a", bestTarget.getPoseAmbiguity());

          if(bestTarget.getDetectedObjectConfidence() >= 0.5){
            estimatedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(), currentCameraTransform);
            
            // Dump information to SmartDashboard. RTE = Robot Transform Estimation. RTE.Rot.X = Robot Transform Estimation's Rotation X. 
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.x", estimatedRobotPose.getX());
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.y", estimatedRobotPose.getY());
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.z", estimatedRobotPose.getZ());
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.rot.x", Units.radiansToDegrees(estimatedRobotPose.getRotation().getX()));
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.rot.y", Units.radiansToDegrees(estimatedRobotPose.getRotation().getY()));
            SmartDashboard.putNumber(currentCamera.getName() + ".rte.rot.z", Units.radiansToDegrees(estimatedRobotPose.getRotation().getZ()));
            SmartDashboard.putString(currentCamera.getName() + ".rte.timestamp", timestamp);
          } else {
            System.out.println("Warning: Reporting inadequate confidence for BestTarget " + bestTarget.fiducialId + " detected from the " + currentCamera + " camera");
          }
        } 
      }
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    dumpCameraData(cameras, cameraTransforms);
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
