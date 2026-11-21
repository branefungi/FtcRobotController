package org.firstinspires.ftc.teamcode.StarterBot;

import android.util.Size;

import com.pedropathing.follower.Follower;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name = "Teleop", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private DrivetrainStarter drivetrain;
    private DcMotorEx intake;
    private CRServo rightIntake;
    private CRServo leftIntake;
    private CRServo transfer;
    private DcMotorEx flyWheel1;
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
    boolean Pressed = false;
    //front left = 0
    //front right = 1
    //back left = 2
    //back right = 3

    //forward = 0, 2
    //backwards = 1, 3
    double Pos = 0.28;
    double turretPos;
    double FWV;
    boolean idle = false;
    double camRange = 0;
    double bearing = 0;
    double elevation = 0;
    double range = 30;
    //private Follower follower;
    //private Pose pose;
    private boolean auto = false;
    private boolean a2Press = false;
    private double rangeRA = 30;
    private double rangeRS = 30;
    private double height;
    private double xPos = 0, yPos = 0; //heading = 0;
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
        
        // initializes movement motors
        drivetrain = new DrivetrainStarter(hardwareMap, gamepad1, telemetry, false, fieldCentric);
        //follower = Constants.createStarter(hardwareMap);
        //follower.setPose(new Pose(0,0,Math.toRadians(0)));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        transfer = hardwareMap.get(CRServo.class, "windmillServo");
        transfer.setDirection(CRServo.Direction.FORWARD);

        rightIntake = hardwareMap.get(CRServo.class, "right_intake_servo");
        rightIntake.setDirection(CRServo.Direction.FORWARD);

        leftIntake = hardwareMap.get(CRServo.class, "left_intake_servo");
        leftIntake.setDirection(CRServo.Direction.FORWARD);

        flyWheel1 = hardwareMap.get(DcMotorEx.class, "launcher");
        flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,fwPID);

        waitForStart();
        opModeTimer.reset();

        while (opModeIsActive()) {

            // update variables

            //follower.update();
            //pose = follower.pose();
//            xPos = pose.x();
//            yPos = pose.y();
//            heading = pose.heading();

            FWV = flyWheel1.getVelocity();
            // set initial values
            if(!auto && !gamepad1.x){
                intakePower = 0;
            }






            // all the movement controls.
            if(!auto){
                drivetrain.Teleop(0);
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


            // apply final values
            intake.setPower(0);
            transfer.setPower(intakePower);
            flyWheel1.setVelocity(FWTarget);

            botTelemetry();




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
            intakePower = 0.5;
        } else if(gamepad1.a){
            intakePower = -0.25;
        }
    }



    public void firing(){
        if(gamepad2.dpad_down && !Pressed){

            Pos-=0.02;
            Pressed = true;
        }
        if(gamepad2.dpad_up && !Pressed){

            Pos+=0.02;

            Pressed = true;
        }
        if(!gamepad2.dpad_down && !gamepad2.dpad_up){
            Pressed = false;
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
                intakePower = Math.min(feedPower + 2 * turretTimer.milliseconds(), feedPower+500);
            }else{
                timerReset = false;
            }
        } else {
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
        telemetry.addData("targetVel", FWTarget);
        telemetry.addData("turretPos", turretPos);
        telemetry.addData("turret speed target", FWVal);


        telemetry.addData("turret target", turretTarget);
        telemetry.update();




    }


}
