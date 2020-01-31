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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Loading Zone Blue", group = "Autonomous")
//@Disabled
public class LZA_BlueDistance extends OpMode {
    public double returnDistance;
    hardwareTOBOR robo = new hardwareTOBOR();
    public double TPI = (28 * 30) / (Math.PI * 4);
    public final int extenderOutCount = 0;
    public vuforiaNav vuNav = new vuforiaNav();
    public ElapsedTime runtime = new ElapsedTime();
    int i;
    boolean drive = false;
    boolean flag = false;
    boolean isFirstStoneDelivered = false;
    int steps = 0;
    double dist = 0;
    double tarDist = 0;
    double SIA = 0;
    int g = 0;

    public enum State {
        TESTSTUFF,
        ARMDOWN,
        SCANQUARRY,
        HUNTSTONE,
        GETLASTSTONE,
        GRABLASTSTONE,
        BACKOFF,
        GRABSTONE,
        HUNTSTONE2,
        ROTATIN,
        DELIVERSTONE,
        RETURN,
        ROTATE,
        PARK,
        STOP

    }

    public State currentState = State.ARMDOWN;
    //public State currentState = State.TESTSTUFF;
    public double lastPosition = 0;

    @Override
    public void init() {
        robo.initHW(hardwareMap);
        vuNav.initVuforiaNav(hardwareMap, true);

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
        robo.resetZAxis(false, false);
        double left = robo.leftish.getDistance(DistanceUnit.INCH);
        double right = robo.rightish.getDistance(DistanceUnit.INCH);
        double back = robo.backish.getDistance(DistanceUnit.INCH);
        telemetry.addData("State", currentState);
        telemetry.addData("Left Reading", left);
        telemetry.addData("Rightish", right);
        telemetry.addData("Back Reading", back);
        telemetry.addData("Drive X", vuNav.driveX);
        telemetry.addData("Drive Y", vuNav.driveY);
        if (right >= 40 || right <= 10 || back > 1.2) {
            telemetry.addData("", "YOU'VE GOT A PROBLEM!!");
        } else {
            telemetry.addData("", "YOU'RE GOOD!!");
        }
        telemetry.addData("Stone Visibile", vuNav.isStoneVisible(true));
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
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("State", currentState);
        switch (currentState) {
            case TESTSTUFF:
                vuNav.targetsSkyStone.activate();
                telemetry.addData("Visible?", vuNav.isStoneVisible(true));
                telemetry.addData("Drive X", vuNav.driveX);
                telemetry.addData("Drive Y", vuNav.driveY);
                telemetry.addData("Leftish", robo.leftish.getDistance(DistanceUnit.INCH));
                telemetry.addData("Rightish", robo.rightish.getDistance(DistanceUnit.INCH));
                telemetry.addData("Backish", robo.backish.getDistance(DistanceUnit.INCH));
                break;
            case ARMDOWN:
                robo.slurp(true);
                if (runtime.seconds() <= 0.3) {

                } else if (runtime.seconds() <= 1) {
                    robo.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robo.armExtend.setPower(0);
                    robo.elbow.setPower(0.5);
                } else {
                    robo.directionLock = -5;
                    robo.arcadeDrive(0, -1, 0, .5);
                    robo.elbow.setPower(0);
                }

                if (robo.backish.getDistance(DistanceUnit.INCH) > 18) {
                    robo.stopMotors();
                    currentState = State.SCANQUARRY;
                    g = 0;
                    robo.directionLock = 0;
                    steps = 0;
                    runtime.reset();
                }
                break;
            case SCANQUARRY:
                if (runtime.seconds() > 1) {
                    if (drive) {
                        steps++;
                    }
                    drive = !drive;
                    runtime.reset();
                }

                if (drive && !isFirstStoneDelivered) {
                    robo.arcadeDrive(-.25, 0, 0, 0.8);
                } else if (drive && isFirstStoneDelivered) {
                    if (steps == 2 || robo.rightish.getDistance(DistanceUnit.INCH) <= 1.5) {
                        currentState = State.GETLASTSTONE;
                        g = 0;
                    } else {
                        robo.arcadeDrive(.25, 0, 0, 0.8);
                    }
                } else {
                    robo.stopMotors();
                }
                vuNav.targetsSkyStone.activate();
                if (vuNav.isStoneVisible(true)) {
                    robo.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    currentState = State.HUNTSTONE;
                    g = 0;
                    robo.stopMotors();
                    runtime.reset();
                }
                break;
            case HUNTSTONE:
                if (!isFirstStoneDelivered) {
                    if (vuNav.isStoneVisible(true)) {
                        if (robo.leftish.getDistance(DistanceUnit.INCH) <= 2 && vuNav.driveX <= -3) {
                            currentState = State.GETLASTSTONE;
                            g = 0;
                        } else {
                            robo.arcadeDrive(vuNav.driveX * 0.2, 0, 0, 0.2);
                            if (Math.abs(vuNav.driveX) <= 0.5) {
                                robo.stopMotors();
                                currentState = State.GRABSTONE;
                                g = 0;
                                returnDistance = robo.backish.getDistance(DistanceUnit.INCH);
                            }
                            runtime.reset();
                            i = 0;
                        }
                    } else {
                        i++;
                        if (i > 10) {
                            robo.arcadeDrive(0, 0, 0, 0.2);
                            if (runtime.seconds() >= 1.4) {
                                returnDistance = robo.backish.getDistance(DistanceUnit.INCH);
                                currentState = State.GRABSTONE;
                                g = 0;
                            }
                        }
                    }
                } else {
                    vuNav.isStoneVisible(true);
                    boolean isLast = robo.rightish.getDistance(DistanceUnit.INCH) <= 2 && vuNav.driveX >= .7;
                    if (vuNav.isStoneVisible(true) && (!isLast)) {
                        robo.arcadeDrive(vuNav.driveX * 0.2, 0, 0, 0.2);
                        if (Math.abs(vuNav.driveX) <= 0.5) {
                            robo.stopMotors();
                            currentState = State.GRABSTONE;
                            runtime.reset();
                            g = 0;
                            returnDistance = robo.backish.getDistance(DistanceUnit.INCH);
                        }
                        runtime.reset();
                        i = 0;
                    } else if (isLast) {
                        currentState = State.GETLASTSTONE;
                    } else {
                        i++;
                        if (i > 10) {
                            robo.arcadeDrive(0, 0, 0, 0.2);
                            if (runtime.seconds() >= 1.4) {
                                returnDistance = robo.backish.getDistance(DistanceUnit.INCH);
                                currentState = State.GRABSTONE;
                                runtime.reset();
                                g = 0;
                            }
                        }
                    }
                }
                break;
            case GETLASTSTONE:
                robo.SLURP.setPosition(.35);
                robo.directionLock = -35;
                if (robo.getIntegratedZAxis() < -31) {
                    currentState = State.GRABLASTSTONE;
                    g = 0;
                    robo.stopMotors();
                    runtime.reset();
                } else {
                    robo.arcadeDrive(0, -.4, 0, 1);
                }
                break;
            case GRABLASTSTONE:
                robo.directionLock = -45;
                robo.arcadeDrive(0, -0.6, 0, 0.5);
                robo.slurp(false);
                if (runtime.seconds() >= 0.8) {
                    currentState = State.BACKOFF;
                    g = 0;
                    runtime.reset();
                }
                break;
            case BACKOFF:
                robo.arcadeDrive(0, 0.5, 0, 0.8);
                if (runtime.seconds() >= .7) {
                    currentState = State.ROTATIN;
                    g = 0;
                }
                break;
            case GRABSTONE:
                robo.arcadeDrive(0, -0.6, 0, 0.9);
                dist = robo.backish.getDistance(DistanceUnit.INCH);
                g++;
                if (g >= 2) {
                    runtime.reset();
                    currentState = State.HUNTSTONE2;
                    g = 0;
                }

                break;
            case HUNTSTONE2:
                robo.slurp(false);
                if (runtime.seconds() > 0.5) {
                    dist = robo.backish.getDistance(DistanceUnit.INCH);
                    if (isFirstStoneDelivered) {
                        tarDist = returnDistance + 8;
                    } else {
                        tarDist = returnDistance + 8;
                    }
                    SIA = dist > (tarDist + 8) ? 1 : Math.abs((dist - tarDist) / 3);

                    robo.arcadeDrive(0, SIA, 0, .8);
                    if (dist <= (tarDist + 0.5)) {
                        currentState = State.ROTATIN;
                        g = 0;
                        robo.stopMotors();
                        runtime.reset();
                    }
                }
                break;
            case ROTATIN:
                robo.directionLock = 90;
                if (robo.getIntegratedZAxis() > 86) {
                    dist = robo.leftish.getDistance(DistanceUnit.INCH);
                    tarDist = 25;
                    SIA = Range.clip(((dist - tarDist) / 3), -1, 1);
                    robo.arcadeDrive(0, SIA, 0, .4);
                    if (Math.abs(dist - tarDist) <= 1) {
                        g++;
                    }
                    if (g >= 3) {
                        currentState = State.DELIVERSTONE;
                        runtime.reset();
                        g = 0;
                    }
                    robo.directionLock = 90;
                } else {
                    robo.arcadeDrive(0, 0, 0, 1);
                }
                break;
            case DELIVERSTONE:
                double pwr = .8;
                double dist2 = robo.backish.getDistance(DistanceUnit.INCH);
                dist = robo.leftish.getDistance(DistanceUnit.INCH);
                tarDist = 22;

                if (dist>=14)
                {
                    SIA = Range.clip(((dist - tarDist) / 3), -1, 1);
                }
                else
                {
                    SIA = 0;
                }

                if (!isFirstStoneDelivered) {
                    if (dist2 < 100 && dist2 > 20) {
                        runtime.reset();
                    }
                } else {
                    if (dist2 < 100) {

                        if (g == 0) {
                            runtime.reset();
                        } else if (dist2 >= 25) {
                            runtime.reset();
                        }
                    }
                    if (dist2 >= 20) {
                        g = 1;
                    }
                }
                if (dist2 >= 35) {
                    robo.SLURP.setPosition(0.6);
                }
                if (runtime.seconds() > .4) {
                    if (isFirstStoneDelivered) {
                        robo.stopMotors();
                        pwr = 0;
                        robo.SLURP.setPosition(0.6);
                        if (runtime.seconds() >= 1.2) {
                            currentState = State.PARK;
                            g = 0;
                            runtime.reset();
                        }
                    } else {
                        robo.stopMotors();
                        pwr = 0;
                        if (runtime.seconds() >= 1.2) {
                            robo.slurp(true);
                            isFirstStoneDelivered = true;
                            currentState = State.RETURN;
                            g = 0;
                            robo.stopMotors();
                            runtime.reset();
                            robo.SLURP.setPosition(0.6);
                        }
                    }
                }
                robo.arcadeDrive(-pwr, SIA*.6, 0, .9);
                break;
            case RETURN:
                dist = robo.leftish.getDistance(DistanceUnit.INCH);
                tarDist = 20.5;
                if (dist>=14)
                {
                    SIA = Range.clip(((dist - tarDist) / 3), -1, 1);
                }
                else
                {
                    SIA = 0;
                }
                robo.arcadeDrive(1, SIA*.6, 0, .9);
                if ((robo.backish.getDistance(DistanceUnit.INCH) <= 24 && robo.backish.getDistance(DistanceUnit.INCH) >= 15 && runtime.seconds() >= 1.3) || runtime.seconds() >= 1.8) {
                    robo.stopMotors();
                    currentState = State.ROTATE;
                    g = 0;
                }
                break;
            case ROTATE:
                robo.slurp(true);
                robo.directionLock = 0;
                robo.arcadeDrive(0, 0, 0, 1);
                if (robo.getIntegratedZAxis() <= 5) {
                    dist = robo.backish.getDistance(DistanceUnit.INCH);
                    tarDist = 23.5;
                    SIA = Range.clip(((dist - tarDist) / 3), -1, 1);
                    robo.arcadeDrive(0, SIA, 0, .275);
                    if (Math.abs(dist - tarDist) <= 1.5) {
                        g++;
                    }
                    if (g >= 3) {
                        currentState = State.SCANQUARRY;
                        steps = 0;
                        g = 0;
                    }
                }
                break;
            case PARK:
                robo.SLURP.setPosition(0.6);
                robo.arcadeDrive(.8, 0, 0, .5);
                if (robo.backish.getDistance(DistanceUnit.INCH) <= 75 && robo.backish.getDistance(DistanceUnit.INCH) >= 35) {
                    g++;
                    if (g >= 2) {
                        currentState = State.STOP;
                        g = 0;
                    }
                }
                break;
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