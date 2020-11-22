package org.firstinspires.ftc.teamcode.Vitals;
import static java.lang.Math.*;

public class MathFunctions {
    public static double[] stickToMotor(double leftStickX, double leftStickY, double rightStickX){
        double r = hypot(-leftStickY, leftStickX);     //polar coordinates for stick powers: r
        double theta = atan2(-leftStickY, leftStickX); //polar coordinates for stick powers: theta
        double PIover4 = PI/4;                         //45 degrees
        double[] prelimPowers = new double[2];         //output array

        theta -= PIover4;                              //subtracting 45 degrees from our theta value

        //PROJECTION FROM CIRCLE SPACE ONTO A SQUARE SPACE
        if((theta >= -PIover4) && (theta < PIover4)){
            //If less than 45 but greater than or equal to -45
            prelimPowers[0] = 1;
            prelimPowers[1]= tan(theta);
        }
        else if((theta >= PIover4) && (theta < (3 * PIover4))){
            //If less than 135 but greater than or equal to 45
            prelimPowers[0] = cot(theta);
            prelimPowers[1] = 1;
        }
        else if((theta >= 3 * PIover4) && (theta < 5 * PIover4)){
            //If less than 225 but greater than or equal to 135
            prelimPowers[0] = - 1;
            prelimPowers[1]= -tan(theta);
        }
        else if((theta >= 5 * PIover4) && (theta < (7 * PIover4))){
            //If less than 360 but greater or equal to than 225
            prelimPowers[0] = -cot(theta);
            prelimPowers[1] = -1;
        }

        prelimPowers[0] *= r;                     //if joystick isn't fully pushed, scales down.
        prelimPowers[1] *= r;                     //if joystick isn't fully pushed, scales down.

        double rotScaling = 1 + abs(rightStickX); //scaling the power if there is rotation

        double[] finalPowers = new double[4];
        finalPowers[0] = (prelimPowers[0] + rightStickX)/rotScaling; //fl
        finalPowers[1] = (prelimPowers[1] - rightStickX)/rotScaling; //fr
        finalPowers[2] = (prelimPowers[1] + rightStickX)/rotScaling; //bl
        finalPowers[3] = (prelimPowers[0] - rightStickX)/rotScaling; //br

        return finalPowers;
    }

    // COT FUNCTION
    public static double cot(double x){
        double y;
        if((x == PI/2) || (x == (3 * PI/2))){
            y = 0;
        }
        else{
            y = 1/tan(x);
        }

        return y;
    }
}

