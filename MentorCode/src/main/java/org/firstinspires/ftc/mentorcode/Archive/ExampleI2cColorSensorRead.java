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


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name = "Read color sensor", group = "Archive")
@Disabled

public class ExampleI2cColorSensorRead extends OpMode {
    int color = 0; //Variable for color data

    byte[] color1Cache; //The read will return an array of bytes. They are stored in this variable

    public static final byte COLOR1ADDRESS = 0x3E;
    public static final int COLOR1_REG_START = 0x04; //Register to start reading
    public static final int COLOR1_READ_LENGTH = 5; //Number of byte to read
    public static final int COLOR_COMMAND_REGISTER = 0x03; //Register to issue commands to the gyro
    public static final byte COLOR_ACTIVE_COMMAND = 0x00;
    public static final byte COLOR_PASSIVE_COMMAND =0x01;

    private I2cAddr color1Addr;
    public I2cDevice color1;
    public I2cDeviceSynch color1Reader;


    @Override
    public void init() {
        color1 = hardwareMap.i2cDevice.get("color1MR");
        color1Addr = I2cAddr.create8bit(COLOR1ADDRESS);
        color1Reader = new I2cDeviceSynchImpl(color1, color1Addr, false);
        color1Reader.engage();
        color1Reader.write8(COLOR_COMMAND_REGISTER, COLOR_PASSIVE_COMMAND);
    }

    @Override
    public void init_loop() {

    }


    @Override
    public void loop() {

        if(gamepad1.a) {
            color1Reader.write8(COLOR_COMMAND_REGISTER, COLOR_ACTIVE_COMMAND);
        }
        else if (gamepad1.b) {
            color1Reader.write8(COLOR_COMMAND_REGISTER, COLOR_PASSIVE_COMMAND);
        }

        color1Cache = color1Reader.read(COLOR1_REG_START, COLOR1_READ_LENGTH);

        color = (int)color1Cache[0];


// send the info back to driver station using telemetry function.

        telemetry.addData("color number byte", color1Cache[0]);
        telemetry.addData("red byte", color1Cache[1]);
        telemetry.addData("color number int", color );
        telemetry.addData("green", color1Cache[2]);
        telemetry.addData("blue", color1Cache[3]);


    }


    @Override
    public void stop() {

    }

}