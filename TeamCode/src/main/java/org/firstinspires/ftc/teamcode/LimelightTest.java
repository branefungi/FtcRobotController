
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "LL Color Detector", group = "Sensor")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, false, false);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double colorBearing = 0;
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    if (cr.getTargetArea() > 0.003 && cr.getTargetArea() < 0.06) {
                        colorBearing = cr.getTargetXDegrees();
                        telemetry.addData("X", colorBearing);
                        telemetry.addData("area", cr.getTargetArea());
                        break;

                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }


            telemetry.update();
            if(gamepad1.y){
                limelight.pipelineSwitch(1);
            }
            if(gamepad1.b){
                limelight.pipelineSwitch(2);
            }
            if(gamepad1.x){
                limelight.pipelineSwitch(3);
            }
            if(gamepad1.a){
                drivetrain.Teleop(0, -gamepad1.left_stick_y, -gamepad1.left_stick_x, colorBearing * -0.027);
            }else{
                drivetrain.Teleop(0);
            }
        }
        limelight.stop();
    }
}
