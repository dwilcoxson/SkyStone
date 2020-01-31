/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.team7234;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "Vuforia Skystone Drive to Target No Bot", group = "Vuforia")
//@Disabled

public class VuforiaSkystoneDriveToTargetNoRobot extends OpMode {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targetImagesList;
    private VuforiaTrackable skystoneTarget;
    private VuforiaTrackableDefaultListener skystoneListener;
    public vuforiaNav vuNav = new vuforiaNav();
    private static final String VUFORIA_KEY = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
    private boolean targetVisible = false;

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix robotLocationTransform;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    double driveDirectionX = 0;          //Direction the robot should move forward
    String driveDirectionY = "";          //Direction the robot should move left or right
    float robotX, robotY, robotZ, robotAngle;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    @Override
    public void init() {
       // vuforiaInit();
        vuNav.initVuforiaNav(hardwareMap);
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        targetImagesList.activate();//Start tracking the data sets we care about.
        robotLocationTransform = skystoneListener.getUpdatedRobotLocation();
    }


    @Override
    public void loop() {
        targetVisible = false;

        if (skystoneListener.isVisible()) {
            robotLocationTransform = skystoneListener.getUpdatedRobotLocation();
            targetVisible = true;
        }

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;

            VectorF trans = lastLocation.getTranslation();

            robotX = trans.get(0);
            robotY = trans.get(1);
            robotZ = trans.get(2);
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            robotAngle = rotation.thirdAngle;

            driveDirectionX = -robotX / mmPerInch;

            if (robotY < -mmPerInch/2) {
                driveDirectionY = "LEFT";
            } else if (robotY > mmPerInch/2) {
                driveDirectionY = "RIGHT";
            } else {
                driveDirectionY = "CENTER";
            }

        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("Robot Angle", robotAngle);
        telemetry.addData("Drive Direction X", driveDirectionX);
        telemetry.addData("Drive Direction Y", driveDirectionY);
        telemetry.addData("RobotX", robotX);
        telemetry.addData("RobotY", robotY);
        telemetry.addData("RobotZ", robotZ);
        telemetry.addData(skystoneTarget.getName(), skystoneListener.isVisible() ? "Visible" : "Not Visible");

    }


    @Override
    public void stop() {
        targetImagesList.deactivate();
    }

    private void vuforiaInit() {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        targetImagesList = vuforia.loadTrackablesFromAsset("Skystone");

        // Setup the targets to be tracked
        skystoneTarget = targetImagesList.get(0);
        skystoneTarget.setName("Skystone Target");
        skystoneTarget.setLocation(createMatrix(8 * mmPerInch, 0, 0, 90, 0 ,-90));

        /*// For convenience, gather together all the trackable objects in one easily-iterable collection
        * This is not needed in this Opmode as we are only tracking one image.  If we were tracking all
        * images in the Skystone game, the two lines of code below would need to be uncommented
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetImagesList);     */

        // Set phone location on robot
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        //
        // IMPORTANT: If you are using a USB WebCam, you must rotate the phone to use the back camera and be in landscape;
        phoneLocation = createMatrix(0, 0, 0, 0, -90, 0);

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        skystoneListener = (VuforiaTrackableDefaultListener) skystoneTarget.getListener();
        skystoneListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }



    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}