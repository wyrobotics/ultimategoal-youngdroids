package org.firstinspires.ftc.teamcode.Vitals;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import static java.lang.Math.*;

public class AutonFunctions extends OdomManager{
    private ElapsedTime et;

    public AutonFunctions(double[] initLoc, double initTheta) {
        super(initLoc, initTheta);
    }

    public void move(double squares) throws NoSuchMethodException {
        double error, lastError = 0, dt = 0.1, t, lastT = 0, P, I, D, errorChange, correction, integral = 0, derivative, angError;

        final double kp = 60000, ki = 0.0, kd = 0.1, angC = 0.1;                                  //HAVE FUN TUNING
        double[] initLoc = getPosition(false);                                        // Double check init loc is correct
        double[] currentLoc;
        error = squares;

        while(abs(error) > 0.15){                                                   // test error
            t = et.milliseconds();
            if(lastT != 0){
                dt = t - lastT;                                               //cannot divide by 0, but will be dividing by dt.
            }
            currentLoc = getPosition(false);
            double distChange = currentLoc[1] - initLoc[1];
            error = squares - distChange;
            errorChange = error - lastError;

            integral += error * dt;
            derivative = errorChange/dt;

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            angError = currentLoc[2] - initLoc[2];                                                  // <- Neg. Err     Pos. Err ->
            double angPower = angC * angError; //in the event that our robot tends to have an angular error. If there is initially no significant error then ur blessed keep it at 0
            double[] powers = new double[] {correction - angPower, correction + angPower, correction - angPower, correction + angPower};
            driveTrain(powers);
            setMotorPower();

            lastT = t;
            lastError = error;


        }
        double[] powers = new double[] {0, 0, 0, 0};
        driveTrain(powers);
        setMotorPower();
        
    }
    
    public void strafe(double squares) throws NoSuchMethodException{ // left is -, right is +
        double error, lastError = 0, dt = 0.1, t, lastT = 0, P, I, D, errorChange, correction, integral = 0, derivative, angError;
        final double kp = 60000, ki = 0.0, kd = 0.1, angC = 0.1;

        double[] initLoc = getPosition(true);
        double[] currentLoc = initLoc;
        double[] movement = new double[currentLoc.length];
        double[] targetLoc = {currentLoc[0] - (squares * cos(currentLoc[2])), //converting to cartesian
                              currentLoc[1] - (squares * sin(currentLoc[2]))};
        
        for(int x = 0; x < targetLoc.length; x++){             //subtracting the target position from the current position to get how much we much gotta go
            movement[x] = targetLoc[x] - currentLoc[x];
        }
        
        error = movement[0];

        while(abs(error)> 0.15){
            t = et.milliseconds();
            error = movement[0];
            errorChange = error - lastError;
            if(lastT != 0){
                dt = t - lastT;
            }
            
            integral += error * dt;
            derivative = errorChange / dt;
            
            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;
            angError = getAngleChange(initLoc[2]); //finding any angular change from the initial angular orientation, immediately setting it to the angular error
            double angPower = angC * angError;     // tune the constant bbygrl

            double[] powers = new double[4];
            powers[0] = correction - angPower;
            powers[1] = (-correction/2) + angPower;
            powers[2] = (-correction/2) - angPower;
            powers[3] = correction + angPower;

            lastT = t;
            lastError = error;

            currentLoc = getPosition(true);
            for(int x = 0; x < targetLoc.length; x++){             //subtracting the target position from the current position to get how much we much gotta go
                movement[x] = targetLoc[x] - currentLoc[x];
            }
        }
        
    }
}
