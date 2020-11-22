package org.firstinspires.ftc.teamcode.Vitals;
import static java.lang.Math.*;

public class OdomManager extends MecanumDrive{
    private double[] currentLocation;
    private double theta;


    private double lastLeft, lastRight;
    final double ticksPerSquare = 10; //INSERT CORRECT NUMBER HERE OR CODE NO WORKY
    final double wheelD = 10;         //DISTANCE BETWEEN THE TWO WHEELS

    public OdomManager(double[] initLoc, double initTheta){
        currentLocation = initLoc;
        theta = initTheta;
    }
    public void updateOdom(boolean strafing) throws NoSuchMethodException {
        if(!strafing) {
            double d1 = distChange(0);                                   //left wheel
            double d2 = distChange(1);                                   //right wheel
            double d = (d1 + d2) / 2;                                       //average distance
            double phi = (d2 - d1) / wheelD;                                //change in orientation

            theta += phi;                                                   //global orientation
            currentLocation[0] += (d * cos(theta));                         //x
            currentLocation[1] += (d * sin(theta));                         //y
        }
        else{
            double d = distChange(0);
            currentLocation[0] += d;
            currentLocation[2] = getAngle();
        }
    }

    public double[] getPosition(boolean strafing) throws NoSuchMethodException {
        this.updateOdom(strafing);
        return new double[] {currentLocation[0], currentLocation[1], theta};
    }

    public double distChange(int n) throws NoSuchMethodException {
        double output = 0;
        if(n == 0) {
            double currentLeft = fl.getCurrentPosition() / ticksPerSquare;   //maybe switch to bl?
            output = currentLeft - lastLeft;
            lastLeft = currentLeft;
        }
        else if(n == 1){
            double currentRight = br.getCurrentPosition() / ticksPerSquare;
            output = currentRight - lastRight;
            lastRight = currentRight;
        } else {
            throw new NoSuchMethodException("distChange Function input error");
        }
        return output;
    }
}   //If this does not work, first try to implement code which will reset encoder values!
