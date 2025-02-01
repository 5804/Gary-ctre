// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Mat.Tuple2;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  // PhotonCamera[] cameras = { // Maybe later
  //   new PhotonCamera("front"),
  //   new PhotonCamera("right"),
  //   new PhotonCamera("back"),
  //   new PhotonCamera("left")
  // };

  PhotonCamera[] cameras = { 
    frontCamera,
    rightCamera,
    backCamera,
    leftCamera
  };

  Transform3d[] cameraTransforms = {
    new Transform3d(0, 0.175, 0.35, new Rotation3d(0, 0, 0)), // Front camera Transform relative to the robot origin
    new Transform3d(0.495, 0, 0.35, new Rotation3d(0, 0, -Math.PI/2)), // Right camera Transform relative to the robot origin
    new Transform3d(0, -0.155, 0.35, new Rotation3d(0, 0, -Math.PI)), // Back camera Transform relative to the robot origin
    new Transform3d(0.485, 0, 0.35, new Rotation3d(0, 0, Math.PI/2)), // Left camera Transform relative to the robot origin
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

  public void dumpCameraData(PhotonCamera[] cameras) {
    int c = 0;
    for (PhotonCamera camera : cameras) {
      List<PhotonPipelineResult> frameResults = camera.getAllUnreadResults();

      // If there are any target results
      if (!frameResults.isEmpty()) {

        // load the target results (we dont know how to pick this index)
        PhotonPipelineResult result = frameResults.get(frameResults.size() - 1);

        // If there are targets captured
        if (result.hasTargets()) {
          // Get the best target
          PhotonTrackedTarget bestTarget = result.getBestTarget();
          // Boolean hasMultitagPose = result.getMultiTagResult().estimatedPose.isPresent; // Test MultiTag implementation

          // Update shuffleboard
          SmartDashboard.putNumber(camera.getName() + "ID", bestTarget.getFiducialId());
          SmartDashboard.putNumber(camera.getName() + "Pitch", bestTarget.getPitch());
          SmartDashboard.putNumber(camera.getName() + "Yaw", bestTarget.getYaw());
          SmartDashboard.putNumber(camera.getName() + "Range", PhotonUtils.calculateDistanceToTargetMeters(
              .275, .65, // Vary targetHeightMeters and cameraHeightmeters to get accurate results.
              Units.degreesToRadians(0), Units.degreesToRadians(bestTarget.getPitch())));
          SmartDashboard.putNumber(camera.getName() + "Ambiguity", bestTarget.getPoseAmbiguity());

          // robotPose is used to store the position of the robot on the field
          Pose3d robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)); 
          // Calculate robot's field relative pose and store it in robot pose
          if (aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId()).get(),
                cameraTransforms[c]); // Gets camera transforms from cameraTransforms using the counting variable c
          }
          SmartDashboard.putNumber("RobotX", robotPose.getX());
          SmartDashboard.putNumber("RobotY", robotPose.getY());
          SmartDashboard.putNumber("RobotZ", robotPose.getZ());

        }/* else {
          // Set shuffleboard values to 0 when no targets are seen
          SmartDashboard.putNumber(camera.getName() + "ID", 0);
          SmartDashboard.putNumber(camera.getName() + "Pitch", 0);
          SmartDashboard.putNumber(camera.getName() + "Yaw", 0);
          SmartDashboard.putNumber(camera.getName() + "Range", 0);
          SmartDashboard.putNumber(camera.getName() + "Ambiguity", 0);
        } */
      }
    }
    c++;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // logCameraData();
    dumpCameraData(cameras);
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
