
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "ThriftyCam Test")
public class WebcamTest extends LinearOpMode {

    // Camera configuration
    private static final String CAMERA_NAME = "Webcam 1";

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Camera exposure and gain
    private static final long EXPOSURE_MS = 10;
    private static final int CAMERA_GAIN = 1;

    // AprilTag configuration
    private static final float APRILTAG_DECIMATION = 2;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ElapsedTime opModeTimer = new ElapsedTime();

    private boolean streamStarted = false;
    private boolean cameraSettingsApplied = false;

    private double camStreamingTime = 0;

    private String exposureStatus = "Not set";
    private String gainStatus = "Not set";

    @Override
    public void runOpMode() {

        initWebcam();

        telemetry.addLine("Initializing ThriftyCam...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Detect when the camera begins streaming
            if (!streamStarted &&
                    visionPortal.getCameraState() ==
                            VisionPortal.CameraState.STREAMING) {

                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }

            // Allow camera time to initialize before changing settings
            if (streamStarted &&
                    !cameraSettingsApplied &&
                    opModeTimer.milliseconds() >
                            camStreamingTime + 500) {

                cameraSettingsApplied = true;
                cameraControls();
            }

            // Display AprilTag information
            telemetryAprilTag();

            // Camera performance
            telemetry.addData(
                    "Vision FPS",
                    "%.1f",
                    visionPortal.getFps()
            );

            telemetry.addData(
                    "Exposure",
                    exposureStatus
            );

            telemetry.addData(
                    "Gain",
                    gainStatus
            );

            telemetry.addData(
                    "Camera State",
                    visionPortal.getCameraState()
            );

            telemetry.update();

            sleep(20);
        }

        // Shut down camera
        visionPortal.close();
    }

    private void initWebcam() {

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(560.8, 732.2, 400.0, 326.0)
                .build();

        aprilTag.setDecimation(APRILTAG_DECIMATION);

        // Create VisionPortal
        VisionPortal.Builder builder =
                new VisionPortal.Builder();

        // Connect ThriftyCam
        builder.setCamera(
                hardwareMap.get(
                        WebcamName.class,
                        CAMERA_NAME
                )
        );

        // Camera resolution
        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));

        // Use MJPEG for reduced USB bandwidth
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Disable preview to reduce processing overhead
        builder.enableLiveView(false);

        // Add AprilTag processor
        builder.addProcessor(aprilTag);

        // Initialize camera
        visionPortal = builder.build();
    }

    public void cameraControls() {

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        // Configure exposure
        if (exposureControl != null && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {

            exposureControl.setMode(ExposureControl.Mode.Manual);

            long minExposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);

            long maxExposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            if (EXPOSURE_MS >= minExposure && EXPOSURE_MS <= maxExposure) {
                boolean success = exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);

                exposureStatus = success ? EXPOSURE_MS + " ms" : "Failed to set";

            } else {
                exposureStatus = "Outside supported range";
            }

        } else {
            exposureStatus = "Manual exposure unavailable";
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}