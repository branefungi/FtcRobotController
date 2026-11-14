package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.GoalPos;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name = "Scuffed Leave", group = "Autos")
public class ScuffedLeave extends OpMode {
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

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

    public void init(){
        goal = createGoal();
        //gatePos = createGatePose();
        // initializes movement motors

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,Math.toRadians(0)));

        intake = hardwareMap.get(DcMotorEx .class, RobotConstants.INTAKE_MOTOR);
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

        stopper = hardwareMap.get(Servo .class, RobotConstants.STOPPER_SERVO);
        stopper.setDirection(Servo.Direction.FORWARD);

        flap = hardwareMap.get(Servo.class, RobotConstants.FLAP_SERVO);
        flap.setDirection(Servo.Direction.FORWARD);


        //initWebcam();

    }


    public void start(){
        opModeTimer.reset();
        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.5);
        BR.setPower(0.5);

    }

    public void loop(){

    }

}
