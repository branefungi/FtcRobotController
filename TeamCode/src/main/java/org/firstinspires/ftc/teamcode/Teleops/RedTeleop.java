package org.firstinspires.ftc.teamcode.Teleops;

import android.util.Size;

import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ApriltagPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Red Teleop", group = "TeleOp")
public class RedTeleop extends LinearOpMode {
    private Drivetrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private Servo turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    boolean fieldCentric = false;
    private ElapsedTime opModeTimer = new ElapsedTime();
    private boolean streamStarted = false;
    private double camStreamingTime;
    private boolean cameraSettingsApplied = false;
    double hOffset = 0;
    double feedPower = 1;

    boolean useWebcam = true;
    double intakePower = 0;
    double FWTarget1 = 0;
    double FWTarget2 = 0;
    double FWVal1 = 1000;
    double FWVal2 = 1000;
    boolean FWPressed = false;
    double turretTarget = 0.5;
    int head = 0;
    //front left = 0
    //front right = 1
    //back left = 2
    //back right = 3

    //forward = 0, 2
    //backwards = 1, 3
    double stopperPos = 0.2;
    double turretPos;
    double FWV1;
    double FWV2;
    boolean idle = false;
    double camRange = 0;
    double bearing = 0;
    double elevation = 0;
    double range = 30;
    protected ApriltagPos tagA;
    protected ApriltagPos tagS;
    private Follower follower;
    private Pose pose;
    private boolean auto = false;
    private boolean a2Press = false;
    private double rangeRA = 30;
    private double rangeRS = 30;
    private double height;
    private double xPos = 0, yPos = 0, heading = 0;
    private double leadK, shootXPos, shootYPos;
    private double headingOffset = 0;
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = RobotConstants.TURRET_MIN_TICKS;
    private final double highLimit = RobotConstants.TURRET_MAX_TICKS;
    private ElapsedTime turretTimer = new ElapsedTime();
    private boolean timerReset = false;


    private ElapsedTime clickCD = new ElapsedTime();
    PIDFCoefficients fwPID = new PIDFCoefficients(RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I, RobotConstants.FLYWHEEL_D, RobotConstants.FLYWHEEL_F);

    protected String[] targetTagId() {
        return new String[]{RobotConstants.TAG_RA, RobotConstants.TAG_RS};
    }

    @Override
    public void runOpMode() {

        tagA = new ApriltagPos(30, 0, 48);
        tagS = new ApriltagPos(100, 0, 24);

        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, false, fieldCentric);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,Math.toRadians(0)));

        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.TRANSFER_MOTOR);
        transfer.setDirection(DcMotorEx.Direction.FORWARD);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL1);
        flyWheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        flyWheel2 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL2);
        flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,fwPID);


        turret = hardwareMap.get(Servo.class, RobotConstants.TURRET_MOTOR);
        turret.setDirection(Servo.Direction.REVERSE);
        turret.setPosition(0.5);

        stopper = hardwareMap.get(Servo.class, RobotConstants.STOPPER_SERVO);
        stopper.setDirection(Servo.Direction.FORWARD);


        initWebcam();
        waitForStart();
        opModeTimer.reset();

        clickCD.reset();

        while (opModeIsActive()) {
            if (!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }

            if (streamStarted && !cameraSettingsApplied && opModeTimer.milliseconds() > camStreamingTime + 500) {
                cameraSettingsApplied = true;
                cameraControls();
            }

            // update variables

            follower.update();
            pose = follower.pose();
            xPos = pose.x();
            yPos = pose.y();
            heading = pose.heading();
            rangeRA = tagA.findRange(xPos, yPos);
            rangeRS = tagS.findRange(xPos, yPos);

            turretPos = turret.getPosition();

            FWV1 = flyWheel1.getVelocity();
            FWV2 = flyWheel2.getVelocity();
            // set initial values
            if(!auto && !gamepad1.x){
                intakePower = 0;
            }


            // all the movement controls.
            if(!auto){
                drivetrain.Teleop(heading);
            }

            //aiming
            if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                List<AprilTagDetection> detectedTags = aprilTag.getDetections();
                aiming(detectedTags);
                if(!tagA.isFound() && !tagS.isFound()){
                    turretTarget = 0.5;
                }
                turret.setPosition(turretTarget);
            }
//            if(clickCD.seconds() > 0.2){
//                if(gamepad2.dpad_right){
//                    turretTarget += 0.01;
//                    clickCD.reset();
//                } else if (gamepad2.dpad_left) {
//                    turretTarget -= 0.01;
//                    clickCD.reset();
//                }
//            }
            turretTarget = Range.clip(turretTarget, 0, 1);
            turret.setPosition(turretTarget);

            // intake controls
            if(!gamepad1.x) {
                setIntakePower();
            }

            // outtake controls
            //setIdlePower();
            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){ // disable if manual intake
                firing();
            }

//            // parking
//            boolean brakeButton = /*gamepad1.x ||*/ (gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7);
//            if(brakeButton){
//                //brake();
//            }


            // apply final values
            intake.setPower(intakePower);
            transfer.setPower(intakePower*1.4);
            flyWheel1.setVelocity(FWTarget1);
            flyWheel2.setVelocity(FWTarget2);
            stopper.setPosition(stopperPos);

            botTelemetry();




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
            exposureControl.setExposure(RobotConstants.CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
    }





    public void setIdlePower(){
        if(gamepad1.dpad_left){
            idle = true;
        }else if(gamepad1.dpad_right){
            idle = false;
        }
    }





    public void setIntakePower(){
        if (gamepad1.left_trigger > 0.1) {
            stopperPos = 0.28;
            intakePower = 0.5;
        } else if(gamepad1.a){
            stopperPos = 0.2;
            intakePower = -0.25;
        }
    }

    public void aiming(List<AprilTagDetection> detectedTags){
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                telemetry.addData("TAGS", ((AprilTagClusterDetection) detection).metadata.name);
                camRange = 0.665 * Math.sqrt(
                        Math.pow(detection.ftcPose.x, 2) +
                                Math.pow(detection.ftcPose.y, 2) +
                                Math.pow(detection.ftcPose.z, 2)
                ) - RobotConstants.CAMERA_FORWARD_OFFSET_IN;
                bearing = -(34.0/30.0) * detection.ftcPose.bearing; //temp change
                elevation =  -detection.ftcPose.elevation + RobotConstants.CAMERA_ANGLE;

                bearing += startingAngle + Math.toDegrees(heading) + (turretPos - 0.5) * (1/RobotConstants.TURRET_TICKS_PER_DEGREE);   // in degrees
                bearing = Math.toRadians(bearing);
                elevation = Math.toRadians(elevation);

                if(((AprilTagClusterDetection) detection).metadata.name.equals(targetTagId()[0])){
                    if(tagA.isFound() && !gamepad2.a){
                        tagA.update(0.2, xPos, yPos, bearing, elevation, camRange);
                    }else{
                        tagA.update(1, xPos, yPos, bearing, elevation, camRange);
                    }
                    if(tagA.status() == ApriltagPos.Status.ACTIVE){
                        tagS.setZ(29);
                        tagS.setStatus(ApriltagPos.Status.INACTIVE);
                    }
                    if(tagA.status() == ApriltagPos.Status.INACTIVE){
                        tagS.setZ(48);
                        tagS.setStatus(ApriltagPos.Status.ACTIVE);
                    }
                    telemetry.addData("Bearing to goal", detection.ftcPose.bearing);

                    continue;
                }
                if(((AprilTagClusterDetection) detection).metadata.name.equals(targetTagId()[1])){
                    if(tagS.isFound() && !gamepad2.a){
                        tagS.update(0.2, xPos, yPos, bearing, elevation, camRange);
                    }else{
                        tagS.update(1, xPos, yPos, bearing, elevation, camRange);
                    }
                    if(tagS.status() == ApriltagPos.Status.ACTIVE){
                        tagA.setZ(29);
                        tagA.setStatus(ApriltagPos.Status.INACTIVE);
                    }
                    if(tagS.status() == ApriltagPos.Status.INACTIVE){
                        tagA.setZ(48);
                        tagA.setStatus(ApriltagPos.Status.ACTIVE);
                    }
                    continue;
                }
            }
        }

        if(tagS.status() == ApriltagPos.Status.ACTIVE){
            turretTarget = tagA.findAngle(xPos, yPos);
            range = tagA.findRange(xPos, yPos);
        }else {
            turretTarget = tagA.findAngle(xPos, yPos);
            range = tagA.findRange(xPos, yPos);
        }

        turretTarget = turretTarget - startingAngle - Math.toDegrees(heading);

        turretTarget = RobotConstants.TURRET_TICKS_PER_DEGREE * (turretTarget) + 0.5; // convert to encoder ticks + test
        if (turretTarget > highLimit + (RobotConstants.TURRET_TICKS_PER_DEGREE) * 30.0) { //wrap angle
            turretTarget -= 360 * RobotConstants.TURRET_TICKS_PER_DEGREE;
        } else if (turretTarget < lowLimit - (RobotConstants.TURRET_TICKS_PER_DEGREE) * 30.0) {
            turretTarget += 360 * RobotConstants.TURRET_TICKS_PER_DEGREE;
        }

        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));


    }


    public void firing(){
        if(clickCD.seconds() > 0.2){
            if(gamepad1.dpad_up){
                FWVal1 += 10;
                clickCD.reset();
            } else if (gamepad1.dpad_down) {
                FWVal1 -= 10;
                clickCD.reset();
            }else if (gamepad1.dpad_right) {
                FWVal2 += 10;
                clickCD.reset();
            }else if (gamepad1.dpad_left) {
                FWVal2 -= 10;
                clickCD.reset();
            }
        }
        FWTarget1 = FWVal1;
        FWTarget2 = FWVal2;
        feedPower = 0.9;
        if (gamepad1.x) {

            //setting target velocity
            if(Math.abs(FWV1) >= Math.abs(FWTarget1) && Math.abs(FWV2) >= Math.abs(FWTarget2)){
                if(timerReset){
                    turretTimer.reset();
                    timerReset = true;
                }
                stopperPos = 0.2;  // open
                intakePower = Math.min(feedPower + 2 * turretTimer.milliseconds(), feedPower+500);
            }else{
                timerReset = false;
            }
        } else {
            stopperPos = 0.28; // closed
            if(idle) {
                FWTarget1 *= 0.75;
                FWTarget2 *= 0.75;
            } else{
                FWTarget1 = 0;
                FWTarget2 = 0;
            }
        }
    }


    public void botTelemetry(){
        telemetry.addData("range", range);
        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("Range RA", rangeRA);
        telemetry.addData("Range RS", rangeRS);
        telemetry.addData("Audience", tagA);
        telemetry.addData("audience status", tagA.status());
        telemetry.addData("Scoring", tagS);
        telemetry.addData("scoring status", tagS.status());
        telemetry.addData("targetVel1", FWVal1);
        telemetry.addData("targetVel2", FWVal2);
        telemetry.addData("speed 1", FWV1);
        telemetry.addData("speed 2", FWV2);
        telemetry.addData("turretPos", turretPos);


        telemetry.addData("intake power", intakePower);
        telemetry.addData("turret target", turretTarget);
        telemetry.addData("Turret Ticks", turret.getPosition());

        telemetry.update();




    }


}
