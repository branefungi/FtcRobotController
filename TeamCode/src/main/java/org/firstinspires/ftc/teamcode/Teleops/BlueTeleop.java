package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.RobotConstants;

@TeleOp(name = "Blue Teleop", group = "TeleOp")
public class BlueTeleop extends RedTeleop{
    @Override
    protected String[] targetTagId(){
        return new String[]{RobotConstants.TAG_BA, RobotConstants.TAG_BS};
    }
}
