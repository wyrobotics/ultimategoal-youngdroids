package org.firstinspires.ftc.teamcode.Meet1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Vitals.MecanumDrive;
import static org.firstinspires.ftc.teamcode.Vitals.MathFunctions.*;
import static org.firstinspires.ftc.teamcode.Vitals.OpModeFunctions.*;

@TeleOp(name="TELEOP", group = "Meet1")
public class OpModeV1 extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();                                                        //New instance of Mecanum Drive

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);                                                                    //Initializing robot hardware

        waitForStart();                                                                             //Wait for start

        while (opModeIsActive()) {                                                                     //Runs OpMode while active
            drive.driveTrain(stickToMotor(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)); //StickToMotor returns array of translated motor powers, which is then set to motor power vars by driveTrain
            drive.setMotorPower();                                                                   //Applies motor power vars to the wheels

            if (gamepad1.a) {
                closeGrabber();
            }
            if (gamepad1.b) {
                knockGoal();
            }
            if (gamepad1.x) {
                armUp();
            }
            if (gamepad1.y) {
                armDown();
            }

        }


    }
}

