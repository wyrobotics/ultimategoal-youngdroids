package org.firstinspires.ftc.teamcode.Vitals;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.teamcode.Vitals.OpModeFunctions.*;

import java.lang.Math.*;

@Disabled
public class MecanumDrive {
    Orientation lastAngle = new Orientation();
    BNO055IMU imu;
    double globalAngle;
    public static Servo mainGrabber, armServo;

    public void init(HardwareMap HM) {
        fl = HM.dcMotor.get("fl"); bl = HM.dcMotor.get("bl"); // Maps all our motors/servos
        fr = HM.dcMotor.get("fr"); br = HM.dcMotor.get("br"); // :(

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER); br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER); bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE); bl.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode      = BNO055IMU.SensorMode.IMU;                                            // might be an issue
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = HM.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        mainGrabber = HM.servo.get("main_servo");
        armServo = HM.servo.get("arm_servo");
        closeGrabber();
    }

    public DcMotor fl, bl, fr, br; // 4 Drive Motors
    public double LFWheelPower, LBWheelPower, RFWheelPower, RBWheelPower; // Power/Position of m/s

    public synchronized void driveTrain(double left_stick_x,double left_stick_y, double right_stick_x) {
        // Math for the mecanum wheels
        LFWheelPower = (-left_stick_y + right_stick_x + left_stick_x);
        LBWheelPower = (-left_stick_y + right_stick_x - left_stick_x);
        RFWheelPower = (-left_stick_y - right_stick_x - left_stick_x);
        RBWheelPower = (-left_stick_y - right_stick_x + left_stick_x);
    }

    public void driveTrain(double[] powers){
        LFWheelPower = powers[0];
        LBWheelPower = powers[1];
        RFWheelPower = powers[2];
        RBWheelPower = powers[3];
    }

    public void setMotorPower(){
        // Sets the power for of all our wheels
        fl.setPower(LFWheelPower); bl.setPower(LBWheelPower);
        fr.setPower(RFWheelPower); br.setPower(RBWheelPower);
    }

    public void resetEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER); br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER); bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngle.firstAngle;
        if(deltaAngle < -180){
            deltaAngle += 360;
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngle = angles;
        return globalAngle;
    }
    public double getAngleChange(double initAng){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - initAng;

        if(deltaAngle < -180){
            deltaAngle += 360;
        }
        else if(deltaAngle > 180){
            deltaAngle -= 360;
        }

        return deltaAngle;
    }
}