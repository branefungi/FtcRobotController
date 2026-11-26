package org.firstinspires.ftc.teamcode.subsystems;

/** Hardware names and tuning shared by every competition teleop and auto. */
public final class RobotConstants {
    private RobotConstants() { }

    public static final String INTAKE_MOTOR = "intake";
    public static final String TRANSFER_MOTOR = "transfer";
    public static final String TURRET_MOTOR = "turret";
    public static final String FLYWHEEL1 = "FW1";
    public static final String FLYWHEEL2 = "FW2";
    public static final String FLYWHEEL_TWO_MOTOR = "FW2";
    public static final String STOPPER_SERVO = "stopper";
    public static final String FLAP_SERVO = "flap";
    public static final String WEBCAM = "Webcam 1";

    public static final double FLYWHEEL_P = 350;
    public static final double FLYWHEEL_I = 0;
    public static final double FLYWHEEL_D = 0;
    public static final double FLYWHEEL_F = 13;
    public static final double TURRET_MIN_TICKS = 0.25;
    public static final double TURRET_MAX_TICKS = 1;
    public static final double TURRET_TICKS_PER_DEGREE = 0.5/180.0;
    public static final double CAMERA_HEIGHT = 12; //camera inches above the ground
    public static final double CAMERA_FORWARD_OFFSET_IN = 0; //offset in direction camera is facing, in front of center is positive
    public static final double CAMERA_ANGLE = 60; // degrees from facing straight forward (angled up is positive)
    public static final int CAMERA_EXPOSURE_MS = 10;
    public static final String TAG_RA = "RED AUDIENCE";
    public static final String TAG_RS = "RED SCORING";
    public static final String TAG_BA = "BLUE AUDIENCE";
    public static final String TAG_BS = "BLUE SCORING";
}