package org.firstinspires.ftc.teamcode.Meet1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Vitals.OpModeFunctions.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import  org.firstinspires.ftc.teamcode.Vitals.MecanumDrive;

public class AutonV1 extends LinearOpMode {
    MecanumDrive md = new MecanumDrive();

    @Override
    public void runOpMode() throws InterruptedException {
        md.init(hardwareMap);

        waitForStart();

        moveForward((float) 2.5);
        knockGoal();

        moveForward((float) .25);
        strafe(-1, 2);
    }
}
