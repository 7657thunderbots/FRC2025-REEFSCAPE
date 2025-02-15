package frc.robot.subsystems.Pipeline;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PipelineSubsystem extends SubsystemBase {
    private PhotonCamera camera;

    public PipelineSubsystem() {
        camera = new PhotonCamera("photonvision");
        secondaryCamera = new PhotonCamera("photonvision2"); // Assuming the second Pi is named "photonvision2"
    }

    public void processPipeline() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Process the targets
            System.out.println("Target detected!");
            // Add your target processing code here
        } else {
            System.out.println("No targets detected.");
        }
    }

    public void processLimeLightPipeline() {
        // Assuming you have a LimeLight camera setup
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double targetVisible = table.getEntry("tv").getDouble(0);

        if (targetVisible == 1.0) {
            // Process the targets
            System.out.println("LimeLight target detected!");
            // Add your LimeLight target processing code here
        } else {
            System.out.println("No LimeLight targets detected.");
        }
    }

    public void processAprilTagPipeline() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Process the AprilTag targets
            System.out.println("AprilTag target detected!");
            result.getTargets().forEach(target -> {
                // Example: Print the ID of the detected AprilTag
                System.out.println("AprilTag ID: " + target.getFiducialId());
                // Add your AprilTag processing code here
            });
        } else {
            System.out.println("No AprilTag targets detected.");
        }
    }
private PhotonCamera secondaryCamera;

public void processSecondaryPipeline() {
    PhotonPipelineResult result = secondaryCamera.getLatestResult();

    if (result.hasTargets()) {
        // Process the targets from the secondary camera
        System.out.println("Secondary camera target detected!");
        // Add your secondary camera target processing code here
    } else {
        System.out.println("No secondary camera targets detected.");
    }
}


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // processPipeline();
        // processLimeLightPipeline();
        // processAprilTagPipeline();
        // processSecondaryPipeline(); // Call the secondary pipeline process
    }
}