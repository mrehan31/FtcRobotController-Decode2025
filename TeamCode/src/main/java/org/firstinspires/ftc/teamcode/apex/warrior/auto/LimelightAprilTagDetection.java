package org.firstinspires.ftc.teamcode.apex.warrior.auto;

//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp(name = "Limelight AprilTag Detection", group = "Sensor")
public class LimelightAprilTagDetection extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight using the name you saved in the configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set the pipeline (0 is default, adjust if you have multiple pipelines)
        limelight.pipelineSwitch(0);

        // Start polling for data
        telemetry.addData(">", "Press Play to start");
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest result from the Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get AprilTag detections
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (fiducialResults != null && !fiducialResults.isEmpty()) {
                    telemetry.addData("AprilTags Detected", fiducialResults.size());

                    // Loop through each detected AprilTag
                    for (LLResultTypes.FiducialResult tag : fiducialResults) {
                        telemetry.addData("---", "---");
                        telemetry.addData("Tag ID", tag.getFiducialId());
                        telemetry.addData("Family", tag.getFamily());
                        
                        // Position in camera frame (pixels)
//                        telemetry.addData("TX (degrees)", "%.2f", tag.getT());
//                        telemetry.addData("TY (degrees)", "%.2f", tag.getTy());
//                        telemetry.addData("TA (area %)", "%.2f", tag.getTa());

//                        // Robot-relative pose (if available)
//                        if (tag.getRobotPoseFieldSpace() != null) {
//                            telemetry.addData("Robot X", "%.2f inches", tag.getRobotPoseFieldSpace().getX());
//                            telemetry.addData("Robot Y", "%.2f inches", tag.getRobotPoseFieldSpace().getY());
//                            telemetry.addData("Robot Rotation", "%.2f deg", tag.getRobotPoseFieldSpace().getYaw());
//                        }
//
//                        // Target-relative pose (camera to tag)
//                        if (tag.getTargetPoseCameraSpace() != null) {
//                            telemetry.addData("Distance X", "%.2f inches", tag.getTargetPoseCameraSpace().getX());
//                            telemetry.addData("Distance Y", "%.2f inches", tag.getTargetPoseCameraSpace().getY());
//                            telemetry.addData("Distance Z", "%.2f inches", tag.getTargetPoseCameraSpace().getZ());
//                        }
                    }
                } else {
                    telemetry.addData("AprilTags", "None detected");
                }

                // Capture latency information
//                telemetry.addData("Pipeline Latency", "%.1f ms", result.getPipelineLatency());
                telemetry.addData("Capture Latency", "%.1f ms", result.getCaptureLatency());

            } else {
                telemetry.addData("Limelight", "No valid data");
            }

            telemetry.update();
            sleep(50); // Small delay to reduce CPU usage
        }

        // Stop the Limelight when done
        limelight.stop();
    }
}
