package org.firstinspires.ftc.teamcode.pedro;

import android.animation.RectEvaluator;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FL");
                c.backLeftName.set("BL");
                c.frontRightName.set("FR");
                c.backRightName.set("BR");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(false);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(3.0); //3 //-5.131
                c.yPodOffset.set(-5.25); //-5.25 //6.028
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3205232086164952);
                Controller secondaryTranslationalForward = Controller.proportional(0.1184247610459254);
                Controller primaryTranslationalLateral = Controller.proportional(0.437511128192277);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16164867135439379);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012319154429561309));
                c.brake.set(Controller.proportionalFeedforward(0.010471281265127112));

                c.headingFeedback.set(Controller.proportional(4.651668850102912));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05196821740697436, 0.004168444920700798));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0951188107031201, 0.05589640512560669));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001295651093632874, 0.0016598395142086069));

                c.maxAchievableForwardVelocity.set(80.155651547117);
                c.maxAchievableStrafeVelocity.set(63.73533433616092);
                c.naturalForwardDeceleration.set(32.44228592717037);
                c.naturalStrafeDeceleration.set(69.91985680872212);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}