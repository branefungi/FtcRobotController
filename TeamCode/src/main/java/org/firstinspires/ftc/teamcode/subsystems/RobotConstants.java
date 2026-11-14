package org.firstinspires.ftc.teamcode.subsystems;

/** Hardware names and tuning shared by every competition teleop and auto. */
public final class RobotConstants {
    private RobotConstants() { }

    public static final String INTAKE_MOTOR = "intake";
    public static final String TRANSFER_MOTOR = "transfer";
    public static final String TURRET_MOTOR = "turret";
    public static final String FLYWHEEL = "FW";
    public static final String FLYWHEEL_TWO_MOTOR = "FW2";
    public static final String STOPPER_SERVO = "stopper";
    public static final String FLAP_SERVO = "flap";
    public static final String WEBCAM = "Webcam 1";

    public static final double FLYWHEEL_P = 350;
    public static final double FLYWHEEL_I = 0;
    public static final double FLYWHEEL_D = 0;
    public static final double FLYWHEEL_F = 13;
    public static final double TURRET_MIN_TICKS = -750;
    public static final double TURRET_MAX_TICKS = 425;
    public static final double TURRET_TICKS_PER_DEGREE = 4.58195833333;
    public static final double CAMERA_FORWARD_OFFSET_IN = 2;
    public static final int CAMERA_EXPOSURE_MS = 10;
    public static final String tagRA = "RED ALLIANCE";
    public static final String tagRS = "RED SCORING";
    public static final String tagBA = "BLUE ALLIANCE";
    public static final String tagBS = "BLUE SCORING";
}