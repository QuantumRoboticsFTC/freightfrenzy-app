package eu.qrobotics.freightfrenzy.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import eu.qrobotics.freightfrenzy.teamcode.util.Alliance;

@TeleOp
public class RedTeleOP extends BaseTeleOP {
    @Override
    Alliance getAlliance() {
        return Alliance.RED;
    }
}
