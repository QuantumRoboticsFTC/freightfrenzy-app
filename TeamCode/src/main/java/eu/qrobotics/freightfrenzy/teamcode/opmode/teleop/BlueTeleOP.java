package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@TeleOp
@Disabled
public class BlueTeleOP extends BaseTeleOP {
    @Override
    Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
