package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Tank", group = "MichaelL")
public class BasicOpMode_DriveTank extends BasicOpMode_Drive {
    @Override
    public void init() {
        super.init();
        setDrivePOV(false);
    }
}
