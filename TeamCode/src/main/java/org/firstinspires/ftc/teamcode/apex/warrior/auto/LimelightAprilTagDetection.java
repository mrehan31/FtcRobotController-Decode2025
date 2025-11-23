package org.firstinspires.ftc.teamcode.apex.warrior.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "LimelightAprilTagDetection", group = "Robot")
public class LimelightAprilTagDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // Set to true if using a webcam (Limelight 3A)

    // The AprilTag processor
    private AprilTagProcessor aprilTag;

    // The Vision Portal
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initVision();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();

                // Push telemetry to the Driver Station
                telemetry.update();

                // Share the CPU between the background vision processor and the OpMode
                sleep(20);
            }
        }

        // Shut down the vision portal
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor and Vision Portal.
     */
    private void initVision() {

        // Create the AprilTag processor by using a builder
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the Vision Portal by using a builder
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera name (hardware map name)
        builder.setCamera(hardwareMap.get(WebcamName.class, "limelight"));

        // Add the AprilTag processor to the portal
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, but don't start camera streaming yet
        visionPortal = builder.build();

        // You can also change the camera resolution here for performance tuning if needed
        // builder.setCameraResolution(new Size(640, 480));
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("FTC AprilTags", "# of Detections: " + currentDetections.size());

        // Iterate through detections and show info
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\nID: %d", detection.id));
                telemetry.addLine(String.format("Range: %.2f inches", detection.ftcPose.range));
                telemetry.addLine(String.format("Bearing: %.2f degrees", detection.ftcPose.bearing));
                telemetry.addLine(String.format("Elevation: %.2f degrees", detection.ftcPose.elevation));
                telemetry.addLine(String.format("X: %.2f Y: %.2f Z: %.2f inches", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            } else {
                telemetry.addLine(String.format("\nID: %d (no metadata)", detection.id));
            }
        }
    }
}
