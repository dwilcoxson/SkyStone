package org.firstinspires.ftc.team535;

import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class hardwareTOBOR {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public Servo tRexArmL = null;
    public Servo tRexArmR = null;
    public DcMotor armExtend = null;
    public DcMotor elbow = null;
    public DcMotor SLED = null;
    public Servo SLURP = null;
    public DistanceSensor rightish = null;
    public DistanceSensor leftish = null;
    public DistanceSensor backish = null;

    public road[] allRoads = new road[10];
    public road targetRoad = null;
    public road currentRoad = null;
    public Pair<Double, Double> onRoadPoint = null;
    public Pair<Double, Double> destinationRoadPoint = null;
    public Pair<Double, Double> nextPoint = null;
    public Pair<Double, Double> currentPoint = null;
    public Pair<Double, Double> targetPoint = null;
    BNO055IMU imu;
    Orientation angles;
    public double integratedZAxis = 0;
    public double lastHeading = 0;
    public double bestDistance;
    public int bestRoad;
    public int bestRoad2;
    public double directionLock;
    public ArrayList<road> intersects = new ArrayList<road>();
    public ArrayList<road> intersectsHere = new ArrayList<road>();
    public ArrayList<road> intersectsThere = new ArrayList<road>();
    public ArrayList<ArrayList<road>> intersectsHereIntersects = new ArrayList<ArrayList<road>>();
    public ArrayList<ArrayList<road>> intersectsThereIntersects = new ArrayList<ArrayList<road>>();
    public ArrayList<route> allRoutes = new ArrayList<>();
    public route finalRoute = null;
    public boolean turnPrevious = false;
    HardwareMap map = null;
    vuforiaNav vuNav = new vuforiaNav();

    public hardwareTOBOR() {
    }

    public void initHW(HardwareMap passMap) {
        map = passMap;
        frontLeft = map.dcMotor.get("fl");
        frontRight = map.dcMotor.get("fr");
        backLeft = map.dcMotor.get("bl");
        backRight = map.dcMotor.get("br");
        tRexArmR = map.servo.get("r hook");
        tRexArmL = map.servo.get("l hook");
        armExtend = map.dcMotor.get("ex");
        elbow = map.dcMotor.get("elbow");
        SLURP = map.servo.get("slurp");
        SLED = map.dcMotor.get("sled");
        rightish = map.get(DistanceSensor.class, "rh");
        backish = map.get(DistanceSensor.class, "bh");
        leftish = map.get(DistanceSensor.class, "lh");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        allRoads[0] = new road(-60, 60, -60, -60, "B");
        allRoads[1] = new road(-37.5, 60, -37.5, -60, "C");
        allRoads[2] = new road(-11, 11, -11, -11, "D");
        allRoads[3] = new road(11, 11, 11, -11, "X");
        allRoads[4] = new road(37.5, 60, 37.5, -60, "Y");
        allRoads[5] = new road(60, 60, 60, -60, "Z");
        allRoads[6] = new road(-60, -60, 60, 60, "V");
        allRoads[7] = new road(-60, 60, 60, -60, "W");
        allRoads[8] = new road(-60, -11, 60, -11, "A");
        allRoads[9] = new road(-60, 11, 60, 11, "O");
    }

    public static String formatRoute(route input) {
        if (input.roads.size() == 0) {
            return "No Data";
        } else if (input.roads.size() == 1) {
            return input.roads.get(0).id;
        } else if (input.roads.size() == 2) {
            return input.roads.get(0).id + ":" + input.roads.get(1).id;
        } else if (input.roads.size() == 3) {
            return input.roads.get(0).id + ":" + input.roads.get(1).id + ":" + input.roads.get(2).id;
        } else if (input.roads.size() == 4) {
            return input.roads.get(0).id + ":" + input.roads.get(1).id + ":" + input.roads.get(2).id + ":" + input.roads.get(3).id;
        } else if (input.roads.size() == 5) {
            return input.roads.get(0).id + ":" + input.roads.get(1).id + ":" + input.roads.get(2).id + ":" + input.roads.get(3).id + ":" + input.roads.get(4).id;
        } else {
            return "Error in route compiling, Route Size = " + input.roads.size();
        }
    }

    public double snap(double input) {
        if (Math.abs(1 - input) <= 0.1) {
            return 1;
        } else if (Math.abs(0 - input) <= 0.1) {
            return 0;
        } else if (Math.abs(-1 - input) <= 0.1) {
            return -1;
        } else {
            return input;
        }
    }

    public void resetZAxis(boolean isColor,boolean isRed) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastHeading = angles.firstAngle;
        if (isColor)
        {
            if (isRed)
            {
               integratedZAxis = -90;
            }
            else
            {
                integratedZAxis = 90;
            }
        }
        else
        {
            integratedZAxis = 0;
        }
    }

    public double getIntegratedZAxis() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaHeading = angles.firstAngle - lastHeading;
        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }
        integratedZAxis += deltaHeading;
        lastHeading = angles.firstAngle;
        return integratedZAxis;
    }

    public Pair<Double, Double> getCurrentPosition() {
        if (vuNav.getLocation()!=null) {
            double x = vuNav.getLocation()[0];
            double y = vuNav.getLocation()[1];
            double z = vuNav.getLocation()[2];
            double xrot = vuNav.getLocation()[3];
            double yrot = vuNav.getLocation()[4];
            double zrot = vuNav.getLocation()[5];
            return new Pair<>(x, y);
        }
        else
        {
            return null;
        }
    }

    public static double hypotenuse(double num1, double num2) {
        return Math.sqrt((num1 * num1) + (num2 * num2));
    }

    public double getAngle(double X, double Y) {
        double formattedAngle;
        double atan2 = Math.atan2(Y, X) + (3 * Math.PI / 2);
        if (atan2 >= 2 * Math.PI) {
            formattedAngle = atan2 - (2 * Math.PI);
        } else {
            formattedAngle = atan2;
        }
        return formattedAngle;
    }

    public void populateRoadVars(Pair<Double, Double> ITargetPoint) {
        currentPoint = getCurrentPosition();
        double currentX = currentPoint.first;
        double currentY = currentPoint.second;
        targetPoint = ITargetPoint;
        double targetX = targetPoint.first;
        double targetY = targetPoint.second;
        for (int i = 0; i < allRoads.length; i++) {
            double currentDistance = hypotenuse(allRoads[i].check(currentX, currentY).first - currentX, allRoads[i].check(currentX, currentY).second - currentY);
            if (i == 0) {
                bestDistance = currentDistance;
                bestRoad = i;
            } else if (currentDistance < bestDistance && allRoads[i] != currentRoad) {
                bestDistance = currentDistance;
                bestRoad = i;
            }
        }
        currentRoad = allRoads[bestRoad];
        onRoadPoint = new Pair<Double, Double>(currentRoad.check(currentX, currentY).first, currentRoad.check(currentX, currentY).second);
        for (int i = 0; i < allRoads.length; i++) {
            double currentDistance = hypotenuse(allRoads[i].check(targetX, targetY).first - targetX, allRoads[i].check(targetX, targetY).second - targetY);
            if (i == 0) {
                bestDistance = currentDistance;
                bestRoad2 = i;
            } else if (currentDistance < bestDistance) {
                bestDistance = currentDistance;
                bestRoad2 = i;
            }
        }
        targetRoad = allRoads[bestRoad2];
        destinationRoadPoint = new Pair<Double, Double>(targetRoad.check(targetX, targetY).first, targetRoad.check(targetX, targetY).second);
        clearAllRoadVars();
        populateRoute();
        finalRoute = getBestRoute();
    }

    public double getAngleToPicture() {
        double currentY = getCurrentPosition().second;
        double currentX = getCurrentPosition().first;
        double distanceToNorth = Math.sqrt(Math.pow(currentX - 0, 2) + Math.pow(currentY - 72, 2));
        double distanceToEast = Math.sqrt(Math.pow(currentX - 72, 2) + Math.pow(currentY - 0, 2));
        double distanceToSouth = Math.sqrt(Math.pow(currentX - 0, 2) + Math.pow(currentY + 72, 2));
        double distanceToWest = Math.sqrt(Math.pow(currentX + 72, 2) + Math.pow(currentY - 0, 2));
        if (distanceToNorth <= distanceToEast && distanceToNorth <= distanceToWest && distanceToNorth <= distanceToSouth) {
            return Math.atan((72 - currentY) / (0 - currentX));
        } else if (distanceToEast <= distanceToNorth && distanceToEast <= distanceToWest && distanceToEast <= distanceToSouth) {
            return Math.atan((0 - currentY) / (72 - currentX));
        } else if (distanceToSouth <= distanceToNorth && distanceToSouth <= distanceToWest && distanceToSouth <= distanceToEast) {
            return Math.atan((-72 - currentY) / (0 - currentX));
        } else {
            return Math.atan((0 - currentY) / (-72 - currentX));
        }
    }

    public double SSS(double n)
    {
        return (n*Math.pow(Math.abs(n),.35));
    }
    //https://forums.parallax.com/discussion/download/79828/ControllingMecanumDrive%255B1%255D.pdf&sa=U&ved=0ahUKEwiX5LzFiNrfAhVswYsKHTofDrwQFggEMAA&client=internal-uds-cse&cx=002870150170079142498:hq1zjyfbawy&usg=AOvVaw19D74YD--M3YmQ2MGd1rTg
    public void arcadeDrive(double LSX, double LSY, double RSX, double speedControl) {
        double ly = -Range.clip(snap(LSX), -1, 1)*speedControl;
        double lx = Range.clip(snap(LSY), -1, 1)*speedControl;

        double turnK = 70;
        if (Math.abs(RSX)>=0.1)
        {
            turnK -=Math.pow(Math.abs(RSX*speedControl),0.5)*45;
        }
        double directionLockAdjust = 0;
//        if (Math.abs(RSX)>=0.1)
//        {
//            lastRSX5 = lastRSX4;
//            lastRSX4 = lastRSX3;
//            lastRSX3 = lastRSX2;
//            lastRSX2 = lastRSX1;
//            lastRSX1 = RSX;
//            turnPrevious = true;
//            directionLockAdjust = SSS(Range.clip(-RSX,-1,1));
//        }
//        else
//        {
//            if (turnPrevious)
//            {
//                double avRSX = (lastRSX1+lastRSX2+lastRSX3+lastRSX4+lastRSX5)/5;
//                directionLock = getIntegratedZAxis()-((avRSX)*15);
//            }
//            turnPrevious = false;
//            directionLockAdjust = -(getIntegratedZAxis() - directionLock) / turnK;
//            if (Math.abs(directionLockAdjust*turnK)<=2)
//            {
//                directionLockAdjust = 0;
//            }
//            directionLockAdjust = SSS(directionLockAdjust);
//        }
        directionLock -= Range.clip(speedControl*RSX,-1,1)*18;
        directionLockAdjust = -(getIntegratedZAxis() - directionLock) / turnK;
        if (Math.abs(directionLockAdjust*turnK)<=(3/turnK))
        {
            directionLockAdjust = 0;
        }
        //directionLockAdjust = SSS(directionLockAdjust);
        if (Math.abs(directionLockAdjust)<=0.08)
        {
            directionLockAdjust =0;
        }

        double frontLeftP = (-hypotenuse(ly, lx) * Math.cos(getAngle(lx, ly) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)))-directionLockAdjust;

        double frontRightP = (hypotenuse(ly, lx) * Math.sin(getAngle(lx, ly) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)))+directionLockAdjust;

        double backLeftP = (hypotenuse(ly, lx) * Math.sin(getAngle(lx, ly) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)))-directionLockAdjust;

        double backRightP = (-hypotenuse(ly, lx) * Math.cos(getAngle(lx, ly) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)))+directionLockAdjust;

        double maxPower = Math.abs(Math.max(backLeftP, Math.max(frontRightP, Math.max(frontLeftP, backRightP))));



        maxPower = maxPower==0? 0.01 : maxPower;

        backLeftP = (backLeftP / maxPower);
        frontRightP = (frontRightP / maxPower);
        frontLeftP = (frontLeftP / maxPower);
        backRightP = (backRightP / maxPower);
        maxPower = maxPower > 1 ? 1 : Math.max(Math.abs(ly), Math.max(Math.abs(lx), Math.abs(directionLockAdjust)));

        backLeftP = (backLeftP) * maxPower;
        frontRightP = (frontRightP) * maxPower;
        frontLeftP = (frontLeftP) * maxPower;
        backRightP = (backRightP) * maxPower;



        frontLeft.setPower(frontLeftP);
        frontRight.setPower(frontRightP);
        backLeft.setPower(backLeftP);
        backRight.setPower(backRightP);
    }

    public double arcadeDriveOLD(double LSX, double LSY, double RSX, double speedControl) {
        double lx = -Range.clip(snap(LSY), -1, 1);
        double ly = Range.clip(snap(LSX), -1, 1);
        double rx = Range.clip (snap(RSX*Math.abs(RSX)),-1,1);
        double directionLockAdjust;
        double zAxis = getIntegratedZAxis();
        double currentHeading;
        if (zAxis>0)
        {
            currentHeading = zAxis%360;
        }
        else
        {
            currentHeading = (-zAxis)%360;
            currentHeading = 360-currentHeading;
            for (int i=0;currentHeading<=0;i++)
            {
                currentHeading += 360;
            }
        }
        if (Math.abs(rx)>=0.1)
        {
            directionLock = zAxis-(10*rx/Math.abs(rx));
            directionLockAdjust = Range.clip(rx,-1,1);
        }
        else
        {
            directionLockAdjust = (zAxis - directionLock) / 30;
            double sign = directionLockAdjust==0? 0: Math.abs(directionLockAdjust)/directionLockAdjust;
            if (Math.abs(directionLockAdjust*15)<=2)
            {
                directionLockAdjust = 0;
            }
//            if (Math.abs(directionLockAdjust*15)>=30)
//            {
//                directionLockAdjust = 0.36*sign;
//            }
        }
        double direction = getAngle(lx,ly)-directionLock;
        double hypotenuse;
        double backLeftP = 0;
        double frontRightP = 0;
        double frontLeftP = 0;
        double backRightP = 0;
        double maxPower;
        if (Math.abs(LSX)>=0.05||Math.abs(LSY)>=0.05) {
            hypotenuse = hypotenuse(ly, lx);
            backLeftP = hypotenuse * Math.cos(direction + (zAxis * Math.PI / 180) + (Math.PI / 4));
            frontRightP = hypotenuse * Math.cos(direction + (zAxis * Math.PI / 180) + (Math.PI / 4));
            frontLeftP = hypotenuse * -Math.sin(direction + (zAxis * Math.PI / 180) + (Math.PI / 4));
            backRightP = hypotenuse * -Math.sin(direction + (zAxis * Math.PI / 180) + (Math.PI / 4));
            maxPower = Math.abs(Math.max(backLeftP, Math.max(frontRightP, Math.max(frontLeftP, backRightP))));

            backLeftP = (backLeftP / maxPower);
            frontRightP = (frontRightP / maxPower);
            frontLeftP = (frontLeftP / maxPower);
            backRightP = (backRightP / maxPower);
            maxPower = maxPower > 1 ? 1 : Math.max(Math.abs(ly), Math.max(Math.abs(lx), Math.abs(rx)));

            backLeftP = (backLeftP) * maxPower;
            frontRightP = (frontRightP) * maxPower;
            frontLeftP = (frontLeftP) * maxPower;
            backRightP = (backRightP) * maxPower;
        }

        

        frontRightP+=directionLockAdjust;
        frontLeftP-=directionLockAdjust;

        maxPower = Math.abs(Math.max(backLeftP,Math.max(frontRightP,Math.max(frontLeftP,backRightP))));
        maxPower = maxPower==0? 0.01 : maxPower;
        backLeftP = (backLeftP/maxPower);
        frontRightP = (frontRightP/maxPower);
        frontLeftP = (frontLeftP/maxPower);
        backRightP = (backRightP/maxPower);

        maxPower = maxPower > 1 ? 1 : maxPower;

        backLeftP = (backLeftP)*maxPower;
        frontRightP = (frontRightP)*maxPower;
        frontLeftP = (frontLeftP)*maxPower;
        backRightP = (backRightP)*maxPower;

        frontLeft.setPower(speedControl*frontLeftP);
        frontRight.setPower(speedControl*frontRightP);
        backLeft.setPower(speedControl*backLeftP);
        backRight.setPower(speedControl*backRightP);
        return 0;
    }

    public boolean autoArcade(Pair<Double, Double> currentPoint, Pair<Double, Double> targetPoint, double directionLock, double speedControl) {
        double currentY = currentPoint.second;
        double currentX = currentPoint.first;
        double targetY = targetPoint.second;
        double targetX = targetPoint.first;
        double y = targetY - currentY;
        double x = targetX - currentX;
        double directionLockAdjust = -(getIntegratedZAxis() - directionLock) / 75;

        double frontLeftP = speedControl * (Range.clip(hypotenuse(y / 8, x / 8), -1, 1)) * Math.sin(getAngle(x, y) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)) + directionLockAdjust;
        double frontRightP = speedControl * (Range.clip(hypotenuse(y / 8, x / 8), -1, 1)) * Math.cos(getAngle(x, y) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)) - directionLockAdjust;
        double backLeftP = speedControl * (Range.clip(hypotenuse(y / 8, x / 8), -1, 1)) * Math.cos(getAngle(x, y) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)) + directionLockAdjust;
        double backRightP = speedControl * (Range.clip(hypotenuse(y / 8, x / 8), -1, 1)) * Math.sin(getAngle(x, y) + (getIntegratedZAxis() * Math.PI / 180) + (Math.PI / 4)) - directionLockAdjust;

        frontLeft.setPower(snap(frontLeftP));
        frontRight.setPower(snap(frontRightP));
        backLeft.setPower(snap(backLeftP));
        backRight.setPower(snap(backRightP));
        if (hypotenuse(x, y) <= 0.5) {
            return true;
        } else {
            return false;
        }
    }

    public boolean checkIfRouteValid(ArrayList<road> check) {
        boolean toReturn = true;
        for (int i = 0; i < allRoutes.size(); i++) {
            if (check == allRoutes.get(i).roads) {
                toReturn = false;
            }
        }
        ArrayList<road> tempList = new ArrayList<>();
        for (int i3 = 0; i3 < check.size(); i3++) {
            tempList.add(check.get(i3));
        }
        if (tempList.size() > 2 && check.size() > 2) {
            for (int i4 = 0; i4 < check.size(); i4++) {
                for (int i5 = 0; i5 < tempList.size(); i5++) {
                    if (check.get(i4) == tempList.get(i5) && i4 != i5) {
                        toReturn = false;
                    }
                }
            }
        }
        if (check.size() > 2) {
            for (int i = 0; i < check.size() - 2; i++) {
                if (hypotenuseP(check.get(i).getIntersect(check.get(i + 1)), check.get(i).getIntersect(check.get(i + 2))) <= 0.01) {
                    toReturn = false;
                }
            }
        }
        return toReturn;
    }

    public void populateRoute() {
        allRoutes.clear();
        boolean isWritable = true;
        ArrayList<road> toWrite = new ArrayList();
        if (targetRoad == currentRoad) {
            toWrite = new ArrayList();
            toWrite.add(currentRoad);
            isWritable = checkIfRouteValid(toWrite);
            if (isWritable) {
                allRoutes.add(new route(toWrite, getCurrentPosition(), onRoadPoint, destinationRoadPoint, targetPoint));
            }
        }
        if (targetRoad.doesIntersect(currentRoad)) {
            toWrite = new ArrayList();
            toWrite.add(currentRoad);
            toWrite.add(targetRoad);
            isWritable = checkIfRouteValid(toWrite);
            if (isWritable) {
                allRoutes.add(new route(toWrite, getCurrentPosition(), onRoadPoint, destinationRoadPoint, targetPoint));
            }
        }
        for (int i = 0; i < allRoads.length; i++) {
            if (allRoads[i].doesIntersect(currentRoad) && allRoads[i] != currentRoad && allRoads[i] != targetRoad) {
                intersects.add(allRoads[i]);
            }
        }
        for (int i = 0; i < intersects.size(); i++) {
            if (intersects.get(i).doesIntersect(targetRoad)) {
                toWrite = new ArrayList();
                toWrite.add(currentRoad);
                toWrite.add(intersects.get(i));
                toWrite.add(targetRoad);
                isWritable = checkIfRouteValid(toWrite);
                if (isWritable) {
                    allRoutes.add(new route(toWrite, getCurrentPosition(), onRoadPoint, destinationRoadPoint, targetPoint));
                }
            }
        }
        intersectsHere = intersects;
        for (int i = 0; i < allRoads.length; i++) {
            if (allRoads[i].doesIntersect(targetRoad) && allRoads[i] != currentRoad && allRoads[i] != targetRoad) {
                intersectsThere.add(allRoads[i]);
            }
        }
        for (int i = 0; i < intersectsHere.size(); i++) {
            for (int i2 = 0; i2 < intersectsThere.size(); i2++) {
                if (intersectsHere.get(i).doesIntersect(intersectsThere.get(i2)) && intersectsHere.get(i) != intersectsThere.get(i2)) {
                    toWrite = new ArrayList();
                    toWrite.add(currentRoad);
                    toWrite.add(intersectsHere.get(i));
                    toWrite.add(intersectsThere.get(i2));
                    toWrite.add(targetRoad);
                    isWritable = checkIfRouteValid(toWrite);
                    if (isWritable) {
                        allRoutes.add(new route(toWrite, getCurrentPosition(), onRoadPoint, destinationRoadPoint, targetPoint));
                    }
                }
            }
        }

        for (int i = 0; i < allRoutes.size(); i++) {
            if (allRoutes.get(i).roads.size() > 4) {
                allRoutes.remove(i);
            }
        }
        for (int i = 0; i < intersectsHere.size(); i++) {
            for (int i2 = 0; i2 < allRoads.length; i2++) {
                if (intersectsHere.get(i).doesIntersect(allRoads[i2])) {
                    intersectsHereIntersects.add(new ArrayList<road>());
                    intersectsHereIntersects.get(i).add(allRoads[i2]);
                }
            }
        }
        for (int i = 0; i < intersectsThere.size(); i++) {
            for (int i2 = 0; i2 < allRoads.length; i2++) {
                if (intersectsThere.get(i).doesIntersect(allRoads[i2])) {
                    intersectsThereIntersects.add(new ArrayList<road>());
                    intersectsThereIntersects.get(i).add(allRoads[i2]);
                }
            }
        }
        for (int hereCounterOuter = 0; hereCounterOuter < intersectsHereIntersects.size(); hereCounterOuter++) {
            for (int thereCounterOuter = 0; thereCounterOuter < intersectsThereIntersects.size(); thereCounterOuter++) {
                for (int hereCounterInner = 0; hereCounterInner < intersectsHereIntersects.get(hereCounterOuter).size(); hereCounterInner++) {
                    for (int thereCounterInner = 0; thereCounterInner < intersectsThereIntersects.get(thereCounterOuter).size(); thereCounterInner++) {
                        if (intersectsHereIntersects.get(hereCounterOuter).get(hereCounterInner) == intersectsThereIntersects.get(thereCounterOuter).get(thereCounterInner)) {
                            toWrite = new ArrayList();
                            toWrite.add(currentRoad);
                            toWrite.add(intersectsHere.get(hereCounterOuter));
                            toWrite.add(intersectsHereIntersects.get(hereCounterOuter).get(hereCounterInner));
                            toWrite.add(intersectsThere.get(thereCounterOuter));
                            toWrite.add(targetRoad);
                            isWritable = checkIfRouteValid(toWrite);
                            if (isWritable) {
                                allRoutes.add(new route(toWrite, getCurrentPosition(), onRoadPoint, destinationRoadPoint, targetPoint));
                            }
                        }
                    }
                }
            }
        }
        for (int i = 0; i < allRoutes.size(); i++) {

        }
    }

    public static double hypotenuseP(Pair<Double, Double> point1, Pair<Double, Double> point2) {
        return hypotenuse(point1.first - point2.first, point1.second - point2.second);
    }

    public route getBestRoute() {
        int bestIndex = 0;
        for (int i = 0; i < allRoutes.size(); i++) {
            if (allRoutes.get(i).getLength() < allRoutes.get(bestIndex).getLength()) {
                bestIndex = i;
            }
        }
        return allRoutes.get(bestIndex);
    }

    public void hooksGrab(int input)
    {
        if (input>=2)
        {
            tRexArmR.setPosition(0.91);
            tRexArmL.setPosition(0.1);
        }
        else if (input==1)
        {
            tRexArmR.setPosition(0.455);
            tRexArmL.setPosition(0.55);
        }
        else
        {
            tRexArmR.setPosition(0);
            tRexArmL.setPosition(1);
        }
    }
    public void stopMotors ()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void clearAllRoadVars() {
        allRoutes.clear();
        finalRoute = null;
    }

    public static String format(double input) {
        if (input % 0.01 > 0.005) {
            return "" + (input - (input % 0.01) + 0.01) + "";
        } else {
            return "" + (input - (input % 0.01)) + "";
        }
    }

    public static String formatPoint(Pair<Double, Double> input) {
        return "(" + format(input.first) + "," + format(input.second) + ")";
    }
    public void slurp (boolean isOpen)
    {
        if (isOpen)
        {
            SLURP.setPosition(1);
        }
        else
        {
            SLURP.setPosition(.1855);
        }
    }



}
