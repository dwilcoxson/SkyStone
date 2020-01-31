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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Drive Forward", group="Autonomous")
//@Disabled
public class driveForward extends OpMode
{
    public double returnDistance;
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
        robo.resetZAxis(false,false);
        telemetry.addData("State", currentState);
        telemetry.addData("Left Reading",robo.leftish.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rightish",robo.rightish.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back Reading",robo.backish.getDistance(DistanceUnit.INCH));
        if (robo.leftish.getDistance(DistanceUnit.INCH)>=200||robo.leftish.getDistance(DistanceUnit.INCH)<=2)
        {
            telemetry.addData("","YOU'VE GOT A PROBLEM!!");
        }
        else
        {
            telemetry.addData("","YOU'RE GOOD!!");
        }

    }

    @Override
    public void start() {
        robo.directionLock = robo.getIntegratedZAxis();
        robo.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robo.hooksGrab(1);
        runtime.reset();
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
                robo.arcadeDrive(0,-.5,0,1);
                telemetry.addData("Left Distance",dist);
                if (runtime.seconds()>=0.8)
                {
                    currentState = State.STOP;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case STOP:
                robo.elbow.setPower(0);
                robo.hooksGrab(0);
                robo.arcadeDrive(0,0,0,1);
                break;
        }
    }

    @Override
    public void stop() {
    }
}