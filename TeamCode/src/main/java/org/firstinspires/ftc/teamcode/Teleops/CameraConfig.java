package org.firstinspires.ftc.teamcode.Teleops;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Camera Config", group = "TeleOp") // SIDE RED/BLUE

public class CameraConfig extends LinearOpMode { // SIDE
    private Drivetrain drivetrain;
    private Follower follower;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ElapsedTime opModeTimer = new ElapsedTime();

    private boolean streamStarted = false;
    private boolean cameraSettingsApplied = false;

    private double camStreamingTime = 0;

    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, true, false);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        initWebcam();

        waitForStart();

        while (opModeIsActive()) {
            if (!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }

            if (streamStarted && !cameraSettingsApplied && opModeTimer.milliseconds() > camStreamingTime + 500) {
                cameraSettingsApplied = true;
                cameraControls();
            }

            follower.update();
            drivetrain.Teleop(0);

            telemetryAprilTag();
            telemetry.addData("X:", follower.pose().x());
            telemetry.addData("Y:", follower.pose().y());
            telemetry.addData("Heading:", follower.pose().heading());
            telemetry.update();
        }
    }


    private void initWebcam() {

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(560.8, 732.2, 400.0, 326.0)
                .build();

        aprilTag.setDecimation(2);

        // Create VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Connect ThriftyCam
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Camera resolution
        builder.setCameraResolution(new Size(640, 480));

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
            exposureControl.setExposure(10, TimeUnit.MILLISECONDS);
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
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",0.665 * detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}