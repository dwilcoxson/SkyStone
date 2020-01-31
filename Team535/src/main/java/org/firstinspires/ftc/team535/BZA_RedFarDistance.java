/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Building Zone Red Distance Far", group="Autonomous")
@Disabled
public class BZA_RedFarDistance extends OpMode
{
    public double returnDistance;
    public double returnCount;
    hardwareTOBOR robo = new hardwareTOBOR();
    public double TPI = (28*30)/(Math.PI*4);
    public final int extenderOutCount = 0;
    public vuforiaNav vuNav = new vuforiaNav();
    public ElapsedTime runtime = new ElapsedTime();
    int i;
    boolean drive = false;
    int steps;
    double dist = 0;
    double tarDist = 0;
    double SIA = 0;
    int g = 0;
    public enum State
    {
        TESTSTUFF,
        DRIVETOFOUNDATION45,
        GRABFOUNDATION,
        PULLFOUNDATION,
        STRAFEOUT,
        DRIVETOBRIDGE,
        ROTATE,
        DRIVETOQUARRY,
        ROTATE2,
        SCANQUARRY,
        HUNTSTONE,
        GRABSTONE,
        HUNTSTONE2,
        ROTATE3,
        DELIVERSTONE,
        SCORESTONE,
        PARK,
        PARK2,
        STOP

    }
    public State currentState = State.DRIVETOFOUNDATION45;
    //public State currentState = State.TESTSTUFF;
    public double lastPosition = 0;
    @Override
    public void init() {
        robo.initHW(hardwareMap);
        vuNav.initVuforiaNav(hardwareMap,true);

        robo.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robo.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robo.hooksGrab(0);
    }

    @Override
    public void init_loop() {
        robo.resetZAxis(false,true);
        telemetry.addData("State", currentState);
        telemetry.addData("Left Reading",robo.leftish.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rightish",robo.rightish.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Reading",robo.backish.getDistance(DistanceUnit.INCH));
        if (robo.rightish.getDistance(DistanceUnit.INCH)>=200||robo.rightish.getDistance(DistanceUnit.INCH)<=2)
        {
            telemetry.addData("","YOU'VE GOT A PROBLEM!!");
        }
        else
        {
            telemetry.addData("","YOU'RE GOOD!!");
        }
        telemetry.addData("Stone Visibile", vuNav.isStoneVisible(false));

    }

    @Override
    public void start() {
        robo.directionLock = robo.getIntegratedZAxis();
        robo.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.hooksGrab(1);
        vuNav.robotLocationTransform = vuNav.stoneListener.getUpdatedRobotLocation();
    }

    @Override
    public void loop() {
        telemetry.addData("State", currentState);
        switch (currentState)
        {
            case TESTSTUFF:
                vuNav.targetsSkyStone.activate();
                telemetry.addData("Visible?", vuNav.isStoneVisible(false));
                telemetry.addData("Drive X",vuNav.driveX);
                telemetry.addData("Drive Y",vuNav.driveY);
                telemetry.addData("Leftish",robo.leftish.getDistance(DistanceUnit.INCH));
                telemetry.addData("Rightish",robo.rightish.getDistance(DistanceUnit.INCH));
                telemetry.addData("Backish",robo.backish.getDistance(DistanceUnit.INCH));
                break;
            case DRIVETOFOUNDATION45:
                dist = robo.rightish.getDistance(DistanceUnit.INCH);
                tarDist = 12.625;
                SIA = dist>(tarDist+8)?1:Math.abs((dist-tarDist)/3);

                robo.arcadeDrive(SIA*(Math.sqrt(2)/2),-SIA*(Math.sqrt(2)/2),0,1);
                telemetry.addData("Right Distance",dist);
                if (dist<=(tarDist+0.5))
                {
                    currentState = State.GRABFOUNDATION;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case GRABFOUNDATION:
                if (runtime.seconds()<=0.25)
                {
                    robo.arcadeDrive(0,-1,0,.4);
                }
                else
                {
                    robo.stopMotors();
                    robo.hooksGrab(2);
                    if (runtime.seconds()>=0.5)
                    {
                        currentState = State.PULLFOUNDATION;
                        runtime.reset();

                    }
                }
                break;
            case PULLFOUNDATION:
                SIA = dist>(tarDist+8)?1:Math.abs((dist-tarDist)/3);
                dist = robo.backish.getDistance(DistanceUnit.INCH);
                tarDist = 16;
                telemetry.addData("Back Distance",dist);
                if (dist<=(tarDist+0.5)||dist>=200||(g!=0))
                {
                    if (g==0)
                    {
                        runtime.reset();
                        g++;
                    }
                    if (runtime.seconds()<=1&&(dist>=1.75))
                    {
                        robo.arcadeDrive(0,1,0,0.2);
                    }
                    else
                    {
                        robo.stopMotors();
                        robo.hooksGrab(0);
                        if (runtime.seconds()>=1.5) {
                            currentState = State.STRAFEOUT;
                        }
                    }
                }
                else
                {
                    robo.arcadeDrive(0,1,0,0.5);
                }
                break;
            case STRAFEOUT:
                dist = robo.rightish.getDistance(DistanceUnit.INCH);
                tarDist = 37;
                SIA = dist<(tarDist-8)?1:Math.abs((dist-tarDist)/3);

                robo.arcadeDrive(-SIA,0,0,1);
                if (dist>=(tarDist+0.5))
                {
                    currentState = State.ROTATE;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case ROTATE:
                robo.directionLock = -100;
                if (robo.getIntegratedZAxis()>=-88)
                {
                    runtime.reset();
                    robo.arcadeDrive(0,0,0,1);
                }
                if (runtime.seconds()>0.1&&runtime.seconds()<=0.7)
                {
                    robo.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robo.armExtend.setPower(0);
                    robo.elbow.setPower(0.7);
                }
                else if (runtime.seconds()>0.7)
                {
                    robo.directionLock = -90;
                    robo.elbow.setPower(0);
                    currentState = State.DRIVETOQUARRY;
                    g=0;
                }
                break;
            case DRIVETOQUARRY:
                robo.slurp(true);
                dist = robo.backish.getDistance(DistanceUnit.INCH);
                if (dist>=28&&dist<=60)
                {
                    g=1;
                }

                if (g==0)
                {
                    robo.arcadeDrive(-1,0,0,.7);
                }
                else {
                    tarDist = 26;
                    SIA = dist > (tarDist + 8) ? 1 : Math.abs((dist - tarDist) / 3);

                    robo.arcadeDrive(-SIA, 0, 0, .7);
                    if (dist <= (tarDist + 0.5)) {
                        currentState = State.ROTATE2;
                        robo.stopMotors();
                        runtime.reset();
                    }
                }
                break;
            case ROTATE2:
                robo.directionLock = 0;
                double rsx = Math.sqrt(Math.abs(robo.getIntegratedZAxis()/30));
                if (robo.getIntegratedZAxis()>=-2)
                {
                    robo.directionLock = 0;
                    currentState = State.SCANQUARRY;
                    runtime.reset();
                }
                break;
            case DRIVETOBRIDGE:
                if (runtime.seconds()<=0.33)
                {
                    robo.arcadeDrive(0,-1,0,.8);
                }
                else {
                    dist = robo.backish.getDistance(DistanceUnit.INCH);
                    tarDist = 20;
                    SIA = dist < (tarDist - 8) ? 1 : Math.abs((dist - tarDist) / 3);

                    robo.arcadeDrive(0, -SIA, 0, .5);
                    if (dist >= (tarDist + 0.5)) {
                        currentState = State.SCANQUARRY;
                        robo.stopMotors();
                        runtime.reset();
                    }
                }
                break;
            case SCANQUARRY:
                if (runtime.seconds()>1)
                {
                    if (drive)
                    {
                        steps++;
                    }
                    drive = !drive;
                    runtime.reset();
                }

                if (drive)
                {
                    robo.arcadeDrive(-.25,0,0,0.8);
                }
                else
                {
                    robo.stopMotors();
                }
                vuNav.targetsSkyStone.activate();
                if (vuNav.isStoneVisible(true))
                {
                    robo.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    currentState = State.HUNTSTONE;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case HUNTSTONE:
                if (vuNav.isStoneVisible(true)) {
                    robo.arcadeDrive(vuNav.driveX * 0.3, -0.8, 0, 0.2);
                    if (vuNav.driveY <= 2.8) {//switched negative
                        robo.stopMotors();
                        currentState = State.GRABSTONE;
                    }
                    runtime.reset();
                    i=0;
                }
                else
                {
                    i++;
                    if (i>10) {
                        robo.arcadeDrive(0, -0.4, 0, 0.2);
                        if (runtime.seconds() >= 1.4) {
                            currentState = State.GRABSTONE;
                        }
                    }
                }
                break;
            case GRABSTONE:
                robo.slurp(false);
                if (runtime.seconds()>=0.2)
                {
                    currentState = State.HUNTSTONE2;
                }
                break;
            case HUNTSTONE2:
                dist = robo.backish.getDistance(DistanceUnit.INCH);
                tarDist = 5;
                SIA = dist>(tarDist+8)?1:Math.abs((dist-tarDist)/3);

                robo.arcadeDrive(0,SIA,0,.2);
                if (dist<=(tarDist+0.5))
                {
                    currentState = State.ROTATE3;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case ROTATE3:
                robo.directionLock = -100;
                robo.arcadeDrive(0,0,0,1);
                if (robo.getIntegratedZAxis()>=-85)
                {
                    runtime.reset();
                }
                else if (runtime.seconds()>=0.8)
                {
                    robo.directionLock = -90;
                    robo.stopMotors();
                    robo.directionLock = robo.getIntegratedZAxis();
                    currentState = State.DELIVERSTONE;
                }
                break;
            case DELIVERSTONE:
                dist = robo.backish.getDistance(DistanceUnit.INCH);
                tarDist = 250;
                SIA = dist<(tarDist-8)?1:Math.abs((dist-tarDist)/3);

                robo.arcadeDrive(SIA,0,0,.7);
                if (dist>=(tarDist+0.5))
                {
                    currentState = State.SCORESTONE;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case SCORESTONE:
                if (runtime.seconds()<=0.5)
                {
                    robo.elbow.setPower(-0.8);
                }
                else if (runtime.seconds()<=1.6)
                {
                    robo.elbow.setPower(0);
                    robo.arcadeDrive(1,0,0,0.5);
                }
                else if (runtime.seconds()<=2.7)
                {
                    if (runtime.seconds()>=2.3)
                    {
                        robo.elbow.setPower(0.4);
                    }
                    robo.stopMotors();
                }
                else
                {
                    robo.slurp(true);
                    lastPosition = robo.frontRight.getCurrentPosition();
                    currentState = State.PARK;
                    robo.elbow.setPower(0);
                    runtime.reset();
                }
                break;
            case PARK:
                if (runtime.seconds()<=1)
                {
                    robo.arcadeDrive(-0.7,0,0,1);
                }
                else
                {
                    currentState = State.STOP;
                }

                break;
//            case PARK:
//                dist = robo.backish.getDistance(DistanceUnit.INCH);
//                if (dist>=28&&dist<=60)
//                {
//                    currentState=State.PARK2;
//                }
//                if (runtime.seconds()>0.5)
//                {
//                    robo.slurp(false);
//                }
//                robo.arcadeDrive(-.5,0,0,1);
//                break;
//            case PARK2:
//                dist = robo.backish.getDistance(DistanceUnit.INCH);
//                if (dist<=42&&dist>=5)
//                {
//                    robo.arcadeDrive(.4,0,0,1);
//                }
//                else
//                {
//                    currentState = State.STOP;
//                }
//                break;
            case STOP:
                robo.elbow.setPower(0);
                robo.hooksGrab(0);
                robo.stopMotors();
                break;
        }
    }

    @Override
    public void stop() {
    }
}