/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

package org.firstinspires.ftc.mentorcode.Archive;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates how to read the FTC Velocity Vortex beacon color
 */
@Autonomous(name = "Color Sensor for Beacon", group = "Archive")
@Disabled
public class ExampleColorSensorReadBeacon extends OpMode {

    ColorSensor colorSensor;
    DeviceInterfaceModule CDI;

    private ElapsedTime runtime = new ElapsedTime();

    float hsvValues[] = {0,0,0};

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        colorSensor.enableLed(false);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() {

        runtime.reset();
    }

    @Override
    public void loop() {
        Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);

        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
            CDI.setLED(0, false);
            CDI.setLED(1, true);
        }

        else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            CDI.setLED(0, true);
            CDI.setLED(1, false);
        }
        else {
            CDI.setLED(0, false);
            CDI.setLED(1, false);
        }

        telemetry.addData("2 Clear", colorSensor.alpha());
        telemetry.addData("3 Red", colorSensor.red());
        telemetry.addData("4 Green", colorSensor.green());
        telemetry.addData("5 Blue", colorSensor.blue());
        telemetry.addData("6 Hue", hsvValues[0]);
    }

    @Override
    public void stop() {}
}
