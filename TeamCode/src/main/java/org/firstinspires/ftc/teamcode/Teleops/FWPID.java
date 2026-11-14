package org.firstinspires.ftc.teamcode.Teleops;

import android.util.Size;

import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "FWPID OP", group = "TeleOp") // SIDE RED/BLUE

public class FWPID extends LinearOpMode { // SIDE
    private Drivetrain drivetrain;
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private DcMotorEx turret;
    private DcMotorEx flyWheel1;
    private DcMotorEx flyWheel2;
    private Servo stopper;
    private Servo flap;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean gainSet = false;
    boolean fieldCentric = false;
    private ElapsedTime opModeTimer = new ElapsedTime();
    private boolean streamStarted = false;
    private double camStreamingTime;

    double hOffset = 0;
    double feedPower = 1;

    boolean useWebcam = false;
    double intakePower = 0;
    double FWTarget = 0;
    double FWVal = 1510;
    //1627 for better first shot right in front
    //1680 to get more balls in with full load
    //1725 for better first shot in corner
    //1790 to get more balls in with full load
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
    double flapPos = 0.2;
    double stopperPos = 0.2;
    double turretPos;
    double FWV1;
    double FWV2;
    double FWV;
    boolean idle = false;
    double camRange = 0;
    double bearing = 0;
    double elevation = 0;
    protected GoalPos goal;

    private Follower follower;
    private Pose pose;
    private boolean auto = false;
    private double gateAngle = gateAngle();
    private boolean a2Press = false;
    private boolean hasEst = false;
    private double range;
    private double height;
    private double xPos = 0, yPos = 0, heading = 0;
    private double leadK, shootXPos, shootYPos;
    private double headingOffset = 0;
    private final double camOffsetX = 2; //inches (not really inches) forward of center
    private final double camOffsetY = 0; //inches (not really inches) right of center
    private final double startingAngle = 0; // angle from straight forward (counterclockwise in degrees)
    private final double lowLimit = RobotConstants.TURRET_MIN_TICKS;
    private final double highLimit = RobotConstants.TURRET_MAX_TICKS;
    private boolean angleFixed = false;

    private ElapsedTime turretTimer = new ElapsedTime();
    private boolean timerReset = false;

    PIDFCoefficients fwPID = new PIDFCoefficients(RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I, RobotConstants.FLYWHEEL_D, RobotConstants.FLYWHEEL_F);

    //** Alliance-specific values overridden by {@link BlueTeleop}. */
    protected GoalPos createGoal() {
        return new GoalPos(30, 50, 15.5);
    }

    protected double gateAngle() {
        return Math.toRadians(30);
    }

    protected String targetAprilTagId() {
        return RobotConstants.tagRA;
    }

    /** Sign applied to the horizontal aiming correction for this alliance. */
    protected double horizontalCorrectionSign() {
        return -1;
    }

    protected double[] hOffsetConstants(){
        return new double[]{1.5,3.0};
    } //2.0,3


    protected boolean showVoltageTelemetry() {
        return false;
    }

    @Override
    public void runOpMode() {
        goal = createGoal();
        //gatePos = createGatePose();
        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, false, fieldCentric);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,Math.toRadians(0)));

        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        transfer = hardwareMap.get(DcMotorEx.class, RobotConstants.TRANSFER_MOTOR);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

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


        //initWebcam();
        waitForStart();
        opModeTimer.reset();

        while (opModeIsActive()) {


            // update variables
/*
            follower.update();
            pose = follower.getPose();
            xPos = pose.getX();
            yPos = pose.getY();
            heading = pose.getHeading();

            leadK = 0.002 * range + 0.2; //0.0033
            shootXPos = follower.getVelocity().getXComponent() * leadK;
            shootYPos = follower.getVelocity().getYComponent() * leadK;

            Range.clip(leadK, 0, 0.4);
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                headingOffset = heading;
            }
            range = goal.findRange(shootXPos + xPos, shootYPos + yPos);

            turretPos = turret.getCurrentPosition();
            FWV1 = flyWheel1.getVelocity();
            FWV2 = flyWheel2.getVelocity();
            FWV = Math.max(FWV1, FWV2);
*/
            FWV = flyWheel1.getVelocity();
            // set initial values
            if(!auto && !gamepad1.x){
                intakePower = 0;
            }

/*
            // configure webcam
            if(!streamStarted && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
                camStreamingTime = opModeTimer.milliseconds();
                streamStarted = true;
            }
            if(!gainSet && streamStarted && opModeTimer.milliseconds() > camStreamingTime + 500){
                gainSet = cameraControls();
            }
*/
            angleFix();


            // all the movement controls.
            if(!auto){
                drivetrain.Teleop(heading);
            }
            /*
            if(gamepad2.x && !FWPressed){
                if(gamepad2.y){
                    turretTarget-=10;
                }
                else{
                    turretTarget-=1;
                }
                FWPressed = true;
            }
            if(gamepad2.b && !FWPressed){
                if(gamepad2.y){
                    turretTarget+=10;
                }
                else{
                    turretTarget+=1;
                }
                FWPressed = true;
            }
            if(!gamepad2.b && !gamepad2.x) {
                FWPressed = false;
            }

             */
            //aiming
            //if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            //  List<AprilTagDetection> detectedTags = aprilTag.getDetections();
            //aiming(detectedTags);
            //turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //turret.setTargetPosition((int) (4.58195833333*turretTarget));
            //}

            // intake controls
            if(!gamepad1.x) {
                setIntakePower();
            }

            // outtake controls
            setIdlePower();
            if(!(gamepad1.left_trigger > 0.1 || gamepad1.a)){ // disable if manual intake
                firing();
            }

            // parking
            boolean brakeButton = /*gamepad1.x ||*/ (gamepad2.right_trigger > 0.7 && gamepad2.left_trigger > 0.7);
            if(brakeButton){
                //brake();
            }


            // apply final values
            intake.setPower(intakePower);
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
            exposureControl.setExposure(10, TimeUnit.MILLISECONDS);
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
        /*
        //        telemetry.addData("xV", follower.getVelocity().getYComponent());
        //        telemetry.addData("yV", follower.getVelocity().getXComponent());
        for (AprilTagDetection detection : detectedTags) {
            if (detection.metadata != null && detection.id == targetAprilTagId()) {
                camRange = detection.ftcPose.range + camOffsetX;
                bearing = detection.ftcPose.bearing;
                elevation = detection.ftcPose.elevation;

                bearing += startingAngle + Math.toDegrees(heading) + turretPos * (1/RobotConstants.TURRET_TICKS_PER_DEGREE);   // in degrees
                bearing = Math.toRadians(bearing);
                elevation = Math.toRadians(elevation);
                if(hasEst){
                    goal.update(0.15, xPos, yPos, bearing, elevation, camRange);
                }else{
                    goal.update(1, xPos, yPos, bearing, elevation, camRange);
                    hasEst = true;
                }

                break;
            }
        }

        //required turret angle
        if(range < 100){
            hOffset = hOffsetConstants()[0];
        } else{
            hOffset = hOffsetConstants()[1];
        }

        */
        /*
        double turretTarget = goal.findAngle(shootXPos + xPos, shootYPos + yPos)
                - startingAngle
                - Math.toDegrees(heading)
                + horizontalCorrectionSign() * Math.toDegrees(Math.atan2(hOffset, range));


        //telemetry.addData("pre wrap target", 976.0 / 180.0 * turretTarget);
        if (turretTarget > highLimit * (1/RobotConstants.TURRET_TICKS_PER_DEGREE) + 30.0) { //wrap angle
            turretTarget -= 360;
        } else if (turretTarget < lowLimit * (1/RobotConstants.TURRET_TICKS_PER_DEGREE) - 30.0) {
            turretTarget += 360;
        }
        turretTarget = RobotConstants.TURRET_TICKS_PER_DEGREE * turretTarget; // convert to encoder ticks
        // hardware limit
        turretTarget = Range.clip(turretTarget, lowLimit, highLimit); //(Math.toDegrees(Math.atan(3.5 / range)));
        */

        if (gamepad2.a){
            goal.update(1, xPos, yPos, bearing, elevation, camRange);
            double turretPower = 0;
            if(!a2Press){
                turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                a2Press = true;
            }
            if(gamepad2.left_bumper){
                turretPower = 0.4;
            } else if (gamepad2.right_bumper){
                turretPower = -0.4;
            }else{
                turretPower = 0;
            }
            if(gamepad2.y){
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turret.setPower(turretPower);
        }else{ // manual aiming
            if(a2Press){

                /*
                switch(head){
                    case 0:
                        turret.setPower(0);
                        break;
                    case 1:
                        turret.setPower(1);
                        break;
                    case 2:
                        turret.setPower(-1);
                        break;
                    case 3:
                        turret.setPower(-2);
                        break;
                }

                 */

                a2Press = false;
            }


            turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            turret.setTargetPosition((int) (4.58195833333*turretTarget));
        }



    }


    public void firing(){
        //setting flap position
        /*
        flapPos = 0.02104 * Math.pow(range - 50, 0.6);
        if(!Double.isFinite(flapPos)){ flapPos = 0;}
        flapPos = Range.clip(flapPos, 0, 0.22);
        flap.setPosition(flapPos);


         */
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
        flapPos = Range.clip(flapPos, 0, 0.38);
        flap.setPosition(flapPos);




/*
        if(range < 100) {
            FWTarget = 0.903*range * 8 + 970 ;  //0.903*range * 8 + 960
            feedPower = 1;
        } else {
            FWTarget = 0.903*range * 7.462 + 970; //0.903*range * 7.462 + 975
            feedPower = 0.75;
        }

 */

        if(gamepad2.x && !FWPressed){
            if(gamepad2.y){
                FWVal-=10;
            }
            else{
                FWVal-=1;
            }
            FWPressed = true;
        }
        if(gamepad2.b && !FWPressed){
            if(gamepad2.y){
                FWVal+=10;
            }
            else{
                FWVal+=1;
            }
            FWPressed = true;
        }
        if(!gamepad2.b && !gamepad2.x){
            FWPressed = false;
        }



        FWTarget = FWVal;
        feedPower = 0.85;
        if (gamepad1.x) {

            //setting target velocity


            if(Math.abs(FWV) >= Math.abs(FWTarget)){

                stopperPos = 0.275;  // open
                intakePower = feedPower;
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

/*
    public void brake(){
        if(!auto){
            PathChain hold = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), new Pose(xPos + 0.01, yPos, heading)))
                    .setConstantHeadingInterpolation(heading)
                    .setBrakingStrength(0.4)
                    .build();
            follower.followPath(hold,1, true);
            auto = true;
        }
    }
*/
//    public void gate(){
//        if(!auto){
//            PathChain gate = follower.pathBuilder()
//                    .addPath(new BezierLine(pose, gatePos))
//                    .setLinearHeadingInterpolation(heading, gatePos.getHeading())
//                    .setBrakingStrength(0.4)
//                    .build();
//            follower.followPath(gate, true);
//            auto = true;
//        }
//    }

    public void botTelemetry(){
        if (showVoltageTelemetry()) {
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                telemetry.addData(sensor.getDeviceName(), sensor.getVoltage());
            }
        }
        //telemetry.addData("Cam Status", visionPortal.getCameraState());
        //telemetry.addData("Cam Setup", gainSet);
        //telemetry.addData("Goal Position", goal);
        //telemetry.addData("Range", range);
        telemetry.addData("targetVel", FWTarget);
        //telemetry.addData("FWV1", FWV1);
        //telemetry.addData("FWV2", FWV2);
        telemetry.addData("Flap", flapPos);
        telemetry.addData("turretPos", turretPos);
        //telemetry.addData("auto", auto);
        telemetry.addData("stopper", stopperPos);

        telemetry.addData("xPos", xPos);
        telemetry.addData("yPos", yPos);

        telemetry.addData("turret speed target", FWVal);

        telemetry.addData("turret target", turretTarget);
        telemetry.addData("Turret Ticks", turret.getCurrentPosition());
        //telemetry.addData("isBusy", follower.isBusy());
        telemetry.update();




    }


}
