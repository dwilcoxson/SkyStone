package org.firstinspires.ftc.team535;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageDecoder;
import android.media.Image;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;


public class vuforiaNav {
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = true;
    public static final String VUFORIA_KEY = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;
    public static final float stoneZ = 2.00f * mmPerInch;
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;
    public static final float bridgeRotZ = 180;
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public boolean targetVisible = false;
    public float phoneXRotate = 0;
    public float phoneYRotate = 0;
    public float phoneZRotate = 0;
    public OpenGLMatrix robotLocationTransform;
    public double driveX;
    public double driveY;
    public double rot;
    VuforiaTrackableDefaultListener stoneListener;

    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables targetsSkyStone;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    Bitmap tempBitmap;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    VuforiaTrackable stoneTarget = null;
    int width = 0;
    int height = 0;
    public WebcamName frontBumperCam = null;


    enum color {
        RED,
        BLUE,
        YELLOW,
        NOT
    }

    public vuforiaNav() {

    }

    public void initVuforiaNav(HardwareMap map, boolean isWebcam) {



        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);//to show on screen put cameraMonitorViewId in those parentheses<<<<

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.useExtendedTracking = false;
        frontBumperCam = map.get(WebcamName.class, "Webcam 1");
        parameters.cameraName = frontBumperCam;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        if (isWebcam)
//        {
//            frontBumperCam = map.get(WebcamName.class, "Webcam 1");
//            parameters.cameraName = frontBumperCam;
//        }
//        else
//        {
//            frontBumperCam = map.get(WebcamName.class, "Webcam 1");
//            parameters.cameraName = frontBumperCam;
//            //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        }
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");
//        allTrackables.addAll(targetsSkyStone);
       //        blueFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
//        blueRearBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
//        redFrontBridge.setLocation(OpenGLMatrix
//                .translation(-bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
//        redRearBridge.setLocation(OpenGLMatrix
//                .translation(bridgeX, -bridgeY, bridgeZ)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
//        red1.setLocation(OpenGLMatrix
//                .translation(quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        red2.setLocation(OpenGLMatrix
//                .translation(-quadField, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        front1.setLocation(OpenGLMatrix
//                .translation(-halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        front2.setLocation(OpenGLMatrix
//                .translation(-halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blue1.setLocation(OpenGLMatrix
//                .translation(-quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        blue2.setLocation(OpenGLMatrix
//                .translation(quadField, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//
//        rear1.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        rear2.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        final float CAMERA_FORWARD_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;                  // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//        }
        stoneListener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();
        stoneListener.setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        targetsSkyStone.activate();
    }

    public double[] getLocation() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            double[] toReturn = new double[6];
            toReturn[0] = -translation.get(1) / mmPerInch;
            toReturn[1] = translation.get(0) / mmPerInch;
            toReturn[2] = translation.get(2) / mmPerInch;
            toReturn[3] = rotation.firstAngle;
            toReturn[4] = rotation.secondAngle;
            toReturn[5] = rotation.thirdAngle;
            return toReturn;
        } else {
            return null;
        }
    }

    public Bitmap capture() {
        vuforia.enableConvertFrameToBitmap();
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    tempBitmap = bitmap;
                }
            }
        }));
        return tempBitmap;
    }

    public Pair<Integer,Integer> getFoundationLocation(Bitmap pureInput, vuforiaNav.color desiredColor, boolean save) {
        Bitmap input = pureInput;
        width = pureInput.getWidth();
        height = pureInput.getHeight();
        int[] pixels = new int[width * height];
        boolean[] boolMap = new boolean[width*height];
        int[] heatMap1 = new int[width*height];
        int[] heatMap2 = new int[width*height];
        int[] heatMap3 = new int[width*height];
        int[] heatMap4 = new int[width*height];
        int[] heatMap5 = new int[width*height];
        int cutoffThreshold = 600;
        input.getPixels(pixels, 0, width, 0, 0, width, height);
        for (int ix = 0; ix < width; ix++) {
            for (int iy = 0; iy < height; iy++) {
                int index = ix + (iy * width);
                color thisColor = color.NOT;
                if (ix >= cutoffThreshold) {
                    int color = pixels[index];
                    double[] tempDoubleArray = convertToHSV(color);
                    thisColor = getVuNavColor(tempDoubleArray);
                }
                if (thisColor==desiredColor)
                {
                    boolMap[index] = true;
                }
                else {
                    boolMap[index] = false;
                }
            }
        }
        heatMap1 = generateHeatMapFromBool(boolMap,width,height,cutoffThreshold);
        heatMap2 = generateHeatMap(heatMap1,width,height,cutoffThreshold);
        heatMap3 = generateHeatMap(heatMap2,width,height,cutoffThreshold);
        heatMap4 = generateHeatMap(heatMap3,width,height,cutoffThreshold);
        heatMap5 = generateHeatMap(heatMap4,width,height,cutoffThreshold);
        int runx = 0;
        int runy = 0;
        int countx = 0;
        int county = 0;
        for (int ix = 0; ix < width; ix++) {
            for (int iy = 0; iy < height; iy++) {
                int index = ix + (iy * width);
                if (ix>=cutoffThreshold)
                {
                    if (heatMap5[index]>=59048)
                    {
                        pixels[index] = Color.rgb(255,0,0);
                        runx+=ix;
                        runy+=iy;
                        countx++;
                        county++;

                    }
                    else
                    {
                        pixels[index] = Color.rgb(0,0,0);
                    }
                }
                else
                {
                    pixels[index] = Color.rgb(0,0,0);
                }
            }
        }
        int centerIX=0;
        int centerIY=0;
        if (countx!=0) {
            centerIX = runx / countx;
        }
        if (county != 0) {
            centerIY = runy / county;
        }
        pixels[(centerIX+1)+((centerIY+1)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX+1)+((centerIY)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX+1)+((centerIY-1)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX)+((centerIY+1)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX)+((centerIY)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX)+((centerIY-1)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX-1)+((centerIY+1)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX-1)+((centerIY)*width)]=Color.rgb(255,255,255);
        pixels[(centerIX-1)+((centerIY-1)*width)]=Color.rgb(255,255,255);
        if (save)
        {
            input.setPixels(pixels, 0, width, 0, 0, width, height);
            if (input != null) {
                File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame.png"));
                try {
                    FileOutputStream outputStream = new FileOutputStream(file);
                    try {
                        input.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                    } finally {
                        outputStream.close();
                    }
                } catch (IOException e) {
                }
            }
        }
        return new Pair<Integer,Integer>(centerIX,centerIY);
    }

    public double[] convertToHSV(int color) {
        double[] toReturn = new double[3];
        int r = (color & 0x00FF0000) >> 16;
        int g = (color & 0x0000FF00) >> 8;
        int b = (color & 0x000000FF);
        double max = Math.max(r, Math.max(g, b));
        double v = max / 255;
        double min = Math.min(r, Math.min(g, b));
        double s = (max - min) / min;
        double h;
        if (r == max) {
            h = ((g - b) / (max - min)) * 60;
        } else if (g == max) {
            h = (2 + ((b - r) / (max - min))) * 60;
        } else {
            h = (4 + ((r - g) / (max - min))) * 60;
        }
        if (h < 0) {
            h += 360;
        }
        toReturn[0] = h;
        toReturn[1] = s;
        toReturn[2] = v;
        return toReturn;
    }

    public vuforiaNav.color getVuNavColor(double[] hsv) {
        double h = hsv[0];
        double s = hsv[1];
        double v = hsv[2];
        color temp = vuforiaNav.color.NOT;
        if (v < 0.2) {
            temp = vuforiaNav.color.NOT;
        } else if (v > 1) {
        } else if (h > 180 && h < 290) {
            if (s > 0.25) {
                temp = vuforiaNav.color.BLUE;
            } else {
                temp = vuforiaNav.color.NOT;
            }

        } else if (h > 345 || h < 15) {
            if (s > 0.45) {
                temp = vuforiaNav.color.RED;
            } else {
                temp = vuforiaNav.color.NOT;
            }
        } else if (h > 35 && h < 85) {
            if (s > 0.5) {
                temp = vuforiaNav.color.YELLOW;
            } else {
                temp = vuforiaNav.color.NOT;
            }
        } else {
            temp = vuforiaNav.color.NOT;
        }
        return temp;
    }

    public int[] generateHeatMap(int[] input, int width, int height,int cutoffThreshold)
    {
        int[] returningHeatMap = new int[width*height];
        for (int ix = 0; ix < width; ix++) {
            for (int iy = 0; iy < height; iy++) {
                int index = ix + (iy * width);
                if (ix >= cutoffThreshold) {
                    boolean onTop = false;
                    boolean onBottom = false;
                    boolean onLeft = false;
                    boolean onRight = false;
                    if (ix==0)
                    {
                        onLeft=true;
                    }
                    else if (ix==width-1)
                    {
                        onRight = true;
                    }
                    if (iy==0)
                    {
                        onTop = true;
                    }
                    else if (iy==height-1)
                    {
                        onBottom = true;
                    }
                    int ul=0;
                    int uc=0;
                    int ur=0;
                    int cl=0;
                    int cc = (input[index]);
                    int cr=0;
                    int dl=0;
                    int dc=0;
                    int dr=0;
                    if (onLeft)
                    {
                        if (onTop)
                        {
                            cr = (input[index+1]);
                            dc = (input[index+width]);
                            dr = (input[index+1+width]);
                        }
                        else if (onBottom)
                        {
                            uc = (input[index-width]);
                            ur = (input[index-width+1]);
                            cr = (input[index+1]);
                        }
                        else
                        {
                            uc = (input[index-width]);
                            ur = (input[index-width+1]);
                            cr = (input[index+1]);
                            dc = (input[index+width]);
                            dr = (input[index+1+width]);
                        }
                    }
                    else if (onRight)
                    {
                        if (onTop)
                        {
                            cl = (input[index-1]);
                            dl = (input[index-1+width]);
                            dc = (input[index+width]);
                        }
                        else if (onBottom)
                        {
                            ul = (input[index-1-width]);
                            uc = (input[index-width]);
                            cl = (input[index-1]);
                        }
                        else
                        {
                            ul = (input[index-1-width]);
                            uc = (input[index-width]);
                            cl = (input[index-1]);
                            dl = (input[index-1+width]);
                            dc = (input[index+width]);
                        }
                    }
                    else
                    {
                        if (onTop)
                        {
                            cl = (input[index-1]);
                            cr = (input[index+1]);
                            dl = (input[(index-1)+width]);
                            dc = (input[index+width]);
                            dr = (input[(index+1)+width]);
                        }
                        else if (onBottom)
                        {
                            ul = (input[(index-1)-width]);
                            uc = (input[index-width]);
                            ur = (input[(index-width)+1]);
                            cl = (input[index-1]);
                            cr = (input[index+1]);
                        }
                        else
                        {
                            ul = (input[(index-1)-width]);
                            uc = (input[index-width]);
                            ur = (input[(index-width)+1]);
                            cl = (input[index-1]);
                            cr = (input[index+1]);
                            dl = (input[(index-1)+width]);
                            dc = (input[index+width]);
                            dr = (input[(index+1)+width]);
                        }
                    }
                    returningHeatMap[index]= ul+uc+ur+cl+cc+cr+dl+dc+dr;
                }
            }
        }
        return returningHeatMap;
    }
    public int[] generateHeatMapFromBool(boolean[] input, int width, int height,int cutoffThreshold)
    {
        int[] returningHeatMap = new int[width*height];
        for (int ix = 0; ix < width; ix++) {
            for (int iy = 0; iy < height; iy++) {
                int index = ix + (iy * width);
                if (ix >= cutoffThreshold) {
                    boolean onTop = false;
                    boolean onBottom = false;
                    boolean onLeft = false;
                    boolean onRight = false;
                    if (ix==0)
                    {
                        onLeft=true;
                    }
                    else if (ix==width-1)
                    {
                        onRight = true;
                    }
                    if (iy==0)
                    {
                        onTop = true;
                    }
                    else if (iy==height-1)
                    {
                        onBottom = true;
                    }
                    int ul=0;
                    int uc=0;
                    int ur=0;
                    int cl=0;
                    int cc = (input[index]?1:0);
                    int cr=0;
                    int dl=0;
                    int dc=0;
                    int dr=0;
                    if (onLeft)
                    {
                        if (onTop)
                        {
                            cr = (input[index+1]?1:0);
                            dc = (input[index+width]?1:0);
                            dr = (input[index+1+width]?1:0);
                        }
                        else if (onBottom)
                        {
                            uc = (input[index-width]?1:0);
                            ur = (input[index-width+1]?1:0);
                            cr = (input[index+1]?1:0);
                        }
                        else
                        {
                            uc = (input[index-width]?1:0);
                            ur = (input[index-width+1]?1:0);
                            cr = (input[index+1]?1:0);
                            dc = (input[index+width]?1:0);
                            dr = (input[index+1+width]?1:0);
                        }
                    }
                    else if (onRight)
                    {
                        if (onTop)
                        {
                            cl = (input[index-1]?1:0);
                            dl = (input[index-1+width]?1:0);
                            dc = (input[index+width]?1:0);
                        }
                        else if (onBottom)
                        {
                            ul = (input[index-1-width]?1:0);
                            uc = (input[index-width]?1:0);
                            cl = (input[index-1]?1:0);
                        }
                        else
                        {
                            ul = (input[index-1-width]?1:0);
                            uc = (input[index-width]?1:0);
                            cl = (input[index-1]?1:0);
                            dl = (input[index-1+width]?1:0);
                            dc = (input[index+width]?1:0);
                        }
                    }
                    else
                    {
                        if (onTop)
                        {
                            cl = (input[index-1]?1:0);
                            cr = (input[index+1]?1:0);
                            dl = (input[(index-1)+width]?1:0);
                            dc = (input[index+width]?1:0);
                            dr = (input[(index+1)+width]?1:0);
                        }
                        else if (onBottom)
                        {
                            ul = (input[(index-1)-width]?1:0);
                            uc = (input[index-width]?1:0);
                            ur = (input[(index-width)+1]?1:0);
                            cl = (input[index-1]?1:0);
                            cr = (input[index+1]?1:0);
                        }
                        else
                        {
                            ul = (input[(index-1)-width]?1:0);
                            uc = (input[index-width]?1:0);
                            ur = (input[(index-width)+1]?1:0);
                            cl = (input[index-1]?1:0);
                            cr = (input[index+1]?1:0);
                            dl = (input[(index-1)+width]?1:0);
                            dc = (input[index+width]?1:0);
                            dr = (input[(index+1)+width]?1:0);
                        }
                    }
                    returningHeatMap[index]= ul+uc+ur+cl+cc+cr+dl+dc+dr;
                }
            }
        }
        return returningHeatMap;
    }

    public boolean isStoneVisible (boolean isRed)
    {
        if (stoneListener.isVisible())
        {
            OpenGLMatrix robotLocationTransform = stoneListener.getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        else
        {
            lastLocation = null;
        }
        if (lastLocation != null) {
            VectorF trans = lastLocation.getTranslation();
            double robotX = trans.get(0);
            double robotY = trans.get(1);
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            if (!isRed)
            {
                rot = rotation.thirdAngle;

                driveY = robotX/mmPerInch;
                driveX = robotY/mmPerInch;
            }
            else
            {
                rot = rotation.thirdAngle;

                driveY = -robotX/mmPerInch;
                driveX =  robotY/mmPerInch;
            }
        }
        else
        {
            driveX=1.23456;
            driveY = 1.23456;
        }
        if (stoneListener.isVisible())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void stopVuforiaNav() {
        targetsSkyStone.deactivate();
    }
}