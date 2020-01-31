package org.firstinspires.ftc.team535;

import android.util.Pair;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

public class route {
    public ArrayList<road> roads;
    public ArrayList<Pair<Double,Double>> points = new ArrayList<>();
    public Pair<Double,Double> startPoint;
    public Pair<Double,Double> onRoadPoint;
    public Pair<Double,Double>destinationRoadPoint;
    public Pair<Double,Double>destinationPoint;
    public double length;

    public route(ArrayList<road> inputRoute,Pair<Double,Double> IstartPoint,Pair<Double,Double> IonRoadPoint,Pair<Double,Double>IdestinationRoadPoint,Pair<Double,Double>IdestinationPoint)
    {
        roads = inputRoute;
        startPoint = IstartPoint;
        onRoadPoint = IonRoadPoint;
        destinationRoadPoint = IdestinationRoadPoint;
        destinationPoint = IdestinationPoint;
        populateRoutePoints();
        length = getLength();
    }
    public void populateRoutePoints() {
        points.clear();
        points.add(startPoint);
        points.add(onRoadPoint);
        if (roads.size() > 1) {
            for (int i = 0; i < roads.size()-1; i++) {
                points.add(roads.get(i).getIntersect(roads.get(i + 1)));
            }
        }
        points.add(destinationRoadPoint);
        points.add(destinationPoint);
        for (int i=0;i<points.size()-1;i++)
        {
            if (hardwareTOBOR.hypotenuseP(points.get(i),points.get(i+1))<=0.001)
            {
                points.remove(i+1);
            }
        }
    }
    public double getLength(){
        double runningTotal = 0;
        for (int i=0;i<points.size()-1;i++)
        {
               runningTotal += hardwareTOBOR.hypotenuseP(points.get(i),points.get(i+1));
        }
        return runningTotal;
    }


}
