package org.firstinspires.ftc.teamcode.Vitals;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import static org.firstinspires.ftc.teamcode.Vitals.MecanumDrive.*;

import static java.lang.Thread.sleep;


public class OpModeFunctions {
    public static final float secondsPerSquare = 10;     //EXPERIMENTALLY FIND HOW LONG IT TAKES TO TRAVERSE ONE SQUARE AT FULL SPEED.
    public static final int closedGrabberDegrees = 85;   //What is the grabber
    public static final int armUpDegrees = 25;
    public static final int armOutDegrees = 25;          //TUNETUNETUNETUNETUNETUNETUNE

    public static void moveForward(float squares) throws InterruptedException {
        int[] powers = new int[] {1, 1, 1, 1};
        float time = secondsPerSquare * squares;
        sleep((long) time);
    }

    public static void strafe(int direction, int squares) throws InterruptedException {    //left is 1, right is 1
        int[] powers = new int[] {1 * direction, -1 * direction, 1 * direction, -1 * direction};
        float time = secondsPerSquare * squares;
        sleep((long) time);
    }

    public static void closeGrabber(){
        mainGrabber.setPosition(closedGrabberDegrees);
    }
    public static void knockGoal(){
        mainGrabber.setPosition(-90);                            //find angle that knocks it. should be easiest i think
    }
    public static void armUp(){
        armServo.setPosition(armUpDegrees);
    }
    public static void armDown(){
        armServo.setPosition(armOutDegrees);
    }

}
