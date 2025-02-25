package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1; 
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision
{
  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  /**
   * Ambiguity defined as a value between (0,1). Used in {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;
  /**
   * Count of times that the odom thinks we're more than 10 meters away from the AprilTag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values())
      {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
      addAprilTagPositioning();
      updateRobotVisionPosition(currentPose.get());
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }
  

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent())
    {
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);
      }
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation())
    {
      Field2d debugField = visionSim.getDebugField();
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> debugField.getObject("VisionEstimation").setPoses());
    }
    return poseEst;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
    return Optional.empty();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList)
    {
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget i : result.getTargets())
        {
          if (i.getFiducialId() == id)
          {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
      // Uncomment to enable opening of camera views
      // try
      // {
      //   Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      //   Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      //   Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      //   e.printStackTrace();
      // }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField()
  {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (Cameras c : Cameras.values())
    {
      if (!c.resultsList.isEmpty())
      {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets())
        {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  
  enum Cameras
  {
    LEFT_CAM("left", new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
             new Translation3d(Units.inchesToMeters(12.056), Units.inchesToMeters(10.981), Units.inchesToMeters(8.44)),
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    RIGHT_CAM("right", new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
              new Translation3d(Units.inchesToMeters(12.056), Units.inchesToMeters(-10.981), Units.inchesToMeters(8.44)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    CENTER_CAM("center", new Rotation3d(0, Units.degreesToRadians(18), 0),
               new Translation3d(Units.inchesToMeters(-4.628), Units.inchesToMeters(-10.687), Units.inchesToMeters(16.129)),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

    public final Alert latencyAlert;
    public PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    private final Transform3d robotToCamTransform;
    public Matrix<N3, N1> curStdDevs;
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
    public PhotonCameraSim cameraSim;
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      camera = new PhotonCamera(name);
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableDrawWireframe(true);
        
      }
    }

    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    public Optional<PhotonPipelineResult> getBestResult()
    {
      if (resultsList.isEmpty())
      {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
      {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0)
        {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    public Optional<PhotonPipelineResult> getLatestResult()
    {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime)
      {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1);
        if (!resultsList.isEmpty())
        {
          updateEstimatedGlobalPose();
        }
      }
    }

    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        curStdDevs = singleTagStdDevs;
      } else
      {
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          numTags++;
          avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
        if (numTags == 0)
        {
          curStdDevs = singleTagStdDevs;
        } else
        {
          avgDist /= numTags;
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
  public Pose2d getEstimatedPose() {

    // Implement the logic to return the estimated pose

    return new Pose2d(); }
    // Placeholder implementation



// /*
//  * MIT License
//  *
//  * Copyright (c) PhotonVision
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  * SOFTWARE.
//  */

//  package frc.robot.subsystems.Vision;

//  import static frc.robot.Constants.Vision.*;
 
//  import edu.wpi.first.math.Matrix;
//  import edu.wpi.first.math.VecBuilder;
//  import edu.wpi.first.math.geometry.Pose2d;
//  import edu.wpi.first.math.geometry.Rotation2d;
//  import edu.wpi.first.math.numbers.N1;
//  import edu.wpi.first.wpilibj.RobotBase;
//  import edu.wpi.first.math.numbers.N3;
//  import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//  import java.util.List;
//  import java.util.Optional;
//  import org.photonvision.EstimatedRobotPose;
//  import org.photonvision.PhotonCamera;
//  import org.photonvision.PhotonPoseEstimator;
//  import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//  import org.photonvision.simulation.PhotonCameraSim;
//  import org.photonvision.simulation.SimCameraProperties;
//  import org.photonvision.simulation.VisionSystemSim;
//  import org.photonvision.targeting.PhotonTrackedTarget;
//  import org.photonvision.targeting.PhotonPipelineResult;
 
//  public class VisionSubsystem {
//      private final PhotonCamera camera;
//      private final PhotonPoseEstimator photonEstimator;
//      private Matrix<N3, N1> curStdDevs;
 
//      // Simulation
//      private PhotonCameraSim cameraSim;
//      private VisionSystemSim visionSim;
 
//      public VisionSubsystem() {
//          camera = new PhotonCamera(kCameraName);
 
//          photonEstimator =
//                  new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
//          photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
//          // ----- Simulation
        //  if (robot.isSimulation()) {
        //      // Create the vision system simulation which handles cameras and targets on the field.
             
        //      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        //      // targets.
        //      //cameraSim = new PhotonCameraSim(camera, cameraProp);
        //      // Add the simulated camera to view the targets on this simulated field.
        //      //visionSim.addCamera(cameraSim, kRobotToCam);
 
        //      //cameraSim.enableDrawWireframe(true);
        //  }
  
 
//      /**
//       * The latest estimated robot pose on the field from vision data. This may be empty. This should
//       * only be called once per loop.
//       *
//       * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
//       * {@link getEstimationStdDevs}
//       *
//       * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
//       *     used for estimation.
//       */
//      public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
//          Optional<EstimatedRobotPose> visionEst = Optional.empty();
//          for (var change : camera.getAllUnreadResults()) {
//              visionEst = photonEstimator.update(change);
//              updateEstimationStdDevs(visionEst, change.getTargets());
 
//              if (RobotBase.isSimulation()) {
//                  visionEst.ifPresentOrElse(
//                          est ->
//                                  getSimDebugField()
//                                          .getObject("VisionEstimation")
//                                          .setPose(est.estimatedPose.toPose2d()),
//                          () -> {
//                              getSimDebugField().getObject("VisionEstimation").setPoses();
//                          });
//              }
//          }
//          return visionEst;
//      }
 
//      /**
//       * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
//       * deviations based on number of tags, estimation strategy, and distance from the tags.
//       *
//       * @param estimatedPose The estimated pose to guess standard deviations for.
//       * @param targets All targets in this camera frame
//       */
//      private void updateEstimationStdDevs(
//              Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
//          if (estimatedPose.isEmpty()) {
//              // No pose input. Default to single-tag std devs
//              curStdDevs = kSingleTagStdDevs;
 
//          } else {
//              // Pose present. Start running Heuristic
//              var estStdDevs = kSingleTagStdDevs;
//              int numTags = 0;
//              double avgDist = 0;
 
//              // Precalculation - see how many tags we found, and calculate an average-distance metric
//              for (var tgt : targets) {
//                  var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//                  if (tagPose.isEmpty()) continue;
//                  numTags++;
//                  avgDist +=
//                          tagPose
//                                  .get()
//                                  .toPose2d()
//                                  .getTranslation()
//                                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
//              }
 
//              if (numTags == 0) {
//                  // No tags visible. Default to single-tag std devs
//                  curStdDevs = kSingleTagStdDevs;
//              } else {
//                  // One or more tags visible, run the full heuristic.
//                  avgDist /= numTags;
//                  // Decrease std devs if multiple targets are visible
//                  if (numTags > 1) estStdDevs = kMultiTagStdDevs;
//                  // Increase std devs based on (average) distance
//                  if (numTags == 1 && avgDist > 4)
//                      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//                  else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
//                  curStdDevs = estStdDevs;
//              }
//          }
//      }

//      public PhotonPipelineResult getLatestResult() {
//         return camera.getLatestResult();
//     }
 
//      /**
//       * Returns the latest standard deviations of the estimated pose from {@link
//       * #getEstimatedGlobalPose()}, for use with {@link
//       * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
//       * only be used when there are targets visible.
//       */
//      public Matrix<N3, N1> getEstimationStdDevs() {
//          return curStdDevs;
//      }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }
 
/**
 * Update the robot's vision position in the simulation field.
 *
 * @param robotPose The current pose of the robot.
 */
public void updateRobotVisionPosition(Pose2d robotPose) {
  if (Robot.isSimulation()) {
    visionSim.update(robotPose);
    visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags( AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
            
  }
 
}



/**
 * Get the ID of the closest AprilTag to the robot, even if it is not visible.
 *
 * @return The ID of the closest AprilTag.
 */
public int getAprilTagIdEvenIfNotVisible() {
  double closestDistance = Double.MAX_VALUE;
  int closestTagId = -1;

  for (int tagId = 1; tagId <= fieldLayout.getTags().size(); tagId++) {
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
    if (tagPose.isPresent()) {
      double distance = PhotonUtils.getDistanceToPose(currentPose.get(), tagPose.get().toPose2d());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTagId = tagId;
      }
    }
  }

  // Post the closest tag ID to the SmartDashboard
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Closest AprilTag ID (Even if not visible)", closestTagId);

  return closestTagId;
}
/**
 * Add AprilTag positioning in the simulation field.
 */
public void addAprilTagPositioning() {
  if (Robot.isSimulation()) {
    visionSim.addAprilTags(fieldLayout);
  }
}
public void periodic() {
  
  addAprilTagPositioning();
  updateRobotVisionPosition(currentPose.get());
  updateVisionField();
  visionSim.update(currentPose.get());
  
  
  if (Robot.isSimulation())
  {
    visionSim = new VisionSystemSim("Vision");
    visionSim.addAprilTags(fieldLayout);


    for (Cameras c : Cameras.values())
    {
      c.addToVisionSim(visionSim);
    }
    
SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim.enableDrawWireframe(true);
        cameraSim.enableDrawWireframe(true);
    openSimCameraViews();
    addAprilTagPositioning();
    updateRobotVisionPosition(currentPose.get());
  }
}
}
