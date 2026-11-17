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
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private Servo stopper;
    private Servo flap;

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
    double FWTarget = 0;
    double FWVal = 1510;
    boolean FWPressed = false;
    double turretTarget = 0;
    int head = 0;
    boolean flapPressed = false;
    //front left = 0
    //front right = 1
    //back left = 2
    //back right = 3

    //forward = 0, 2
    //backwards = 1, 3
    double flapPos = 0.28;
    double stopperPos = 0.2;
    double turretPos;
    double FWV;
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
    private boolean angleFixed = false;

    private ElapsedTime turretTimer = new ElapsedTime();
    private boolean timerReset = false;

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

        flyWheel1 = hardwareMap.get(DcMotorEx.class, RobotConstants.FLYWHEEL);
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,fwPID);


        turret = hardwareMap.get(DcMotorEx.class, RobotConstants.TURRET_MOTOR);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        turret.setPositionPIDFCoefficients(15);

        stopper = hardwareMap.get(Servo.class, RobotConstants.STOPPER_SERVO);
        stopper.setDirection(Servo.Direction.FORWARD);

        flap = hardwareMap.get(Servo.class, RobotConstants.FLAP_SERVO);
        flap.setDirection(Servo.Direction.FORWARD);


        initWebcam();
        waitForStart();
        opModeTimer.reset();

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

            turretPos = turret.getCurrentPosition();

            FWV = flyWheel1.getVelocity();
            // set initial values
            if(!auto && !gamepad1.x){
                intakePower = 0;
            }




            angleFix();


            // all the movement controls.
            if(!auto){
                drivetrain.Teleop(heading);
            }

            //aiming
            if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                List<AprilTagDetection> detectedTags = aprilTag.getDetections();
                aiming(detectedTags);
                if(!tagA.isFound() && !tagS.isFound()){
                    turretTarget = 0;
                }
                turret.setTargetPosition((int) (turretTarget));
            }

            // intake controls
            if(!gamepad1.x) {
                setIntakePower();
            }

            // outtake controls
            setIdlePower();
            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){ // disable if manual intake
                firing();
            }

//            // parking
//            boolean brakeButton = /*gamepad1.x ||*/ (gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7);
//            if(brakeButton){
//                //brake();
//            }


            // apply final values
            flap.setPosition(flapPos);
            intake.setPower(0);
            transfer.setPower(intakePower*1.4);
            flyWheel1.setVelocity(FWTarget);
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
            stopperPos = 0.2;
            intakePower = 0.5;
        } else if(gamepad1.a){
            stopperPos = 0.275;
            intakePower = -0.25;
        }
    }

    public void angleFix(){
        // lock angle
        if(gamepad1.left_bumper) {
            angleFixed = true;
        }else if(Math.abs(gamepad1.right_stick_x) > 0.1){
            angleFixed = false;
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
                bearing = detection.ftcPose.elevation;
                elevation = -detection.ftcPose.bearing + RobotConstants.CAMERA_ANGLE;


                bearing += startingAngle + Math.toDegrees(heading) + turretPos * (1/RobotConstants.TURRET_TICKS_PER_DEGREE);   // in degrees
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
        if (turretTarget > highLimit * (1/RobotConstants.TURRET_TICKS_PER_DEGREE) + 30.0) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < lowLimit * (1/RobotConstants.TURRET_TICKS_PER_DEGREE) - 30.0) {
            turretTarget += 360;
        }
        turretTarget = RobotConstants.TURRET_TICKS_PER_DEGREE * (turretTarget); // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));


    }


    public void firing(){
        if(gamepad2.dpad_down && !flapPressed){

            flapPos-=0.02;
            flapPressed = true;
        }
        if(gamepad2.dpad_up && !flapPressed){

            flapPos+=0.02;

            flapPressed = true;
        }
        if(!gamepad2.dpad_down && !gamepad2.dpad_up){
            flapPressed = false;
        }

        FWTarget = FWVal;
        feedPower = 0.85;
        if (gamepad1.x) {

            //setting target velocity
            if(Math.abs(FWV) >= Math.abs(FWTarget)){
                if(timerReset){
                    turretTimer.reset();
                    timerReset = true;
                }
                stopperPos = 0.275;  // open
                intakePower = Math.min(feedPower + 2 * turretTimer.milliseconds(), feedPower+500);
            }else{
                timerReset = false;
            }
        } else {
            stopperPos = 0.2; // closed
            if(idle) {
                FWTarget *= 0.75;
            } else{
                FWTarget = 0;
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
        telemetry.addData("targetVel", FWTarget);
        telemetry.addData("turretPos", turretPos);
        telemetry.addData("turret speed target", FWVal);


        telemetry.addData("turret target", turretTarget);
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        telemetry.update();




    }


}
