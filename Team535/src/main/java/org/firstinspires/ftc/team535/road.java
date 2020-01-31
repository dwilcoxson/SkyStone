package org.firstinspires.ftc.team535;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class road {
    public double X1;
    public double Y1;
    public double X2;
    public double Y2;
    public double maxX;
    public double maxY;
    public double minX;
    public double minY;
    public double slope;
    public double YIntercept;
    public String id;
    double intersectX;
    double intersectY;
    boolean XClear;
    boolean YClear;
    boolean isVertical;

    public road(double x1, double y1, double x2, double y2, String inputId)
    {
        X1= Range.clip(x1,-72,72);
        Y1= Range.clip(y1,-72,72);
        X2= Range.clip(x2,-72,72);
        Y2= Range.clip(y2,-72,72);
        id = inputId;
        if (X1!=X2)
        {
            slope = (Y2-Y1)/(X2-X1);
            YIntercept = Y1-(slope*X1);
        }
        else
        {
            isVertical = true;
        }

        if (X1>=X2)
        {
            maxX=X1;
            minX=X2;
        }
        else
        {
            maxX=X2;
            minX=X1;
        }

        if (Y1>=Y2)
        {
            maxY=Y1;
            minY=Y2;
        }
        else
        {
            maxY=Y2;
            minY=Y1;
        }
    }
    //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    public Pair<Double,Double> getCoordinates (double currentX, double currentY) {
        Pair<Double,Double> toReturn = new Pair<Double, Double>(0.0,0.0);

        double idealMagnitude = Math.abs(((Y2-Y1)*currentX)-((X2-X1)*currentY)+(X2*Y1)-(Y2*X1))/Math.sqrt(Math.pow(Y2-Y1,2)+Math.pow(X2-X1,2));
        double lineSlope = (Y2-Y1)/(X2-X1);
        double idealDirection = Math.atan(-1/lineSlope);
        double idealY = (Math.sin(idealDirection)*idealMagnitude)+currentY;
        double idealX = (Math.cos(idealDirection)*idealMagnitude)+currentX;
        boolean XClear;
        boolean YClear;
        if ((idealY<Y2 && idealY>Y1)||(idealY>Y2 && idealY<Y1)||(idealY==Y1&&idealY==Y2))
        {
            YClear = true;
        }
        else
        {
            YClear = false;
        }

        if ((idealX<X2 && idealX>X1)||(idealX>X2 && idealX<X1)||(idealX==X1&&idealX==X2))
        {
            XClear = true;
        }
        else
        {
            XClear = false;
        }

        if (XClear&&YClear)
        {
            toReturn = new Pair<Double, Double>(currentX+idealX,currentY+idealY);
        }
        else if (Math.sqrt(Math.pow(Y1-currentY,2)+Math.pow(X1-currentX,2))<=Math.sqrt(Math.pow(Y2-currentY,2)+Math.pow(X2-currentX,2)))
        {
            toReturn = new Pair<Double, Double>(X1,Y1);
        }
        else
        {
            toReturn = new Pair<Double, Double>(X2,Y2);
        }
        return toReturn;
    }
    public Pair<Double, Double> check(double currentX, double currentY) { //RETURNS DISTANCE TO POINT + IF POINT IS THE IDEAL ONE
        Pair<Double,Double> currentPoint = new Pair<>(currentX,currentY);
        Pair<Double,Double> idealIntersect;
        Pair<Double,Double> nearEndpoint;
        if (isVertical)
        {
            idealIntersect = new Pair<>(this.X1,currentY);
        }
        else if (slope==0)
        {
            idealIntersect = new Pair<>(currentX,this.Y1);
        }
        else
        {
            //mx+b=mx+b
            double rightSlope=-1/this.slope;
            double rightYInt=currentY-(rightSlope*currentX);
            double tempX=(rightYInt-this.YIntercept)/(this.slope-rightSlope);
            double tempY=this.YIntercept+(this.slope*tempX);
            idealIntersect = new Pair<>(tempX,tempY);
        }

        Pair<Double,Double> ep1=new Pair<Double,Double>(X1,Y1);
        Pair<Double,Double> ep2=new Pair<Double,Double>(X2,Y2);
        if (hardwareTOBOR.hypotenuseP(currentPoint,ep1)<=hardwareTOBOR.hypotenuseP(currentPoint,ep2))
        {
            nearEndpoint=ep1;
        }
        else
        {
            nearEndpoint = ep2;
        }
        intersectX = (idealIntersect.first);
        intersectY = (idealIntersect.second);
        if ((intersectY<Y2 && intersectY>Y1)||(intersectY>Y2 && intersectY<Y1)||(Math.abs(intersectY-Y1)<=0.5&&Math.abs(intersectY-Y2)<=0.5))
        {
            YClear = true;
        }
        else
        {
            YClear = false;
        }

        if ((intersectX<X2 && intersectX>X1)||(intersectX>X2 && intersectX<X1)||(Math.abs(intersectX-X1)<=0.5&&Math.abs(intersectX-X2)<=0.5))
        {
            XClear = true;
        }
        else
        {
            XClear = false;
        }

        if (XClear&&YClear)
        {
            return idealIntersect;
        }
        else
        {
            return nearEndpoint;
        }
    }
    public boolean doesIntersect (road otherRoad) {
        boolean XOverlaps = (this.X1-otherRoad.X1)*(this.X2-otherRoad.X2)<=0;
        boolean YOverlaps = (this.Y1-otherRoad.Y1)*(this.Y2-otherRoad.Y2)<=0;
        boolean X1Match = this.X1 == otherRoad.X1;
        boolean X2Match = this.Y2==otherRoad.Y2;
        boolean Y1Match = this.Y1==otherRoad.Y1;
        boolean Y2Match = this.X2==otherRoad.X2;
        double crossY = getIntersect(otherRoad).second;
        double crossX = getIntersect(otherRoad).first;
        if (this == otherRoad)
        {
            return false;
        }
        else if ((X1Match&&Y1Match)||(X2Match&&Y2Match)||(this.X1 == otherRoad.X2&&this.Y1==otherRoad.Y2)||(this.X2 == otherRoad.X1&&this.Y2==otherRoad.Y1))
        {
            return true;
        }
        else if ((crossX<=this.maxX+0.01&&crossX>=this.minX-0.01)&&(crossY<=this.maxY+0.01&&crossY>=this.minY-0.01)&&(crossX<=otherRoad.maxX+0.01&&crossX>=otherRoad.minX-0.01)&&(crossY<=otherRoad.maxY+0.01&&crossY>=otherRoad.minY-0.01))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    public Pair<Double,Double> getIntersect (road otherRoad){
        double x;
        double y;
        if (this.isVertical)
        {
            x = this.X1;
            y = (otherRoad.slope * x)+otherRoad.YIntercept;
        }
        else if (otherRoad.isVertical){
            x = otherRoad.X1;
            y = (this.slope * x)+this.YIntercept;
        }
        else {
            x = (this.YIntercept - otherRoad.YIntercept) / (otherRoad.slope - this.slope);
            y = (this.slope * x) + this.YIntercept;
        }
        return new Pair<Double,Double>(x,y);
    }

}
