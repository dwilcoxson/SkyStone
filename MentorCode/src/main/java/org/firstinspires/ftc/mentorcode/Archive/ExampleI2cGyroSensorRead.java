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

package org.firstinspires.ftc.mentorcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name = "Read MRgyro sensor", group = "Archive")
@Disabled

public class ExampleI2cGyroSensorRead extends OpMode {
    int heading = 0;        //Variable for Heading data
    int intZValue = 0;      //Variable for the integrated Z value

    byte[] gyro1Cache;         //The read will return an array of bytes.  They are stored in this variable

    public static final byte GYRO1ADDRESS = 0x20;           //Default I2C address for MR gyro
    public static final int GYRO1_REG_START = 0x04;         //Register to start reading
    public static final int GYRO1_READ_LENGTH = 14;         //Number of byte to read
    public static final int GYRO_COMMAND_REGISTER = 0x03;   //Register to issue commands to the gyro
    public static final byte GYRO_CALIBRATE_COMMAND = 0x4E; //Command to calibrate the gyro

    private I2cAddr gyro1Addr;
    public I2cDevice gyro1;
    public I2cDeviceSynch gyro1Synch;


    @Override
    public void init() {
        gyro1 = hardwareMap.i2cDevice.get("Gyro");
        gyro1Addr = I2cAddr.create8bit(GYRO1ADDRESS);
        gyro1Synch = new I2cDeviceSynchImpl(gyro1, gyro1Addr, false);
        gyro1Synch.engage();
    }

    @Override
    public void init_loop() {
        //Calibrate the gyro if it is not reading 0
        if (gyro1Synch.read8(4) != 0) {
            gyro1Synch.write8(GYRO_COMMAND_REGISTER, GYRO_CALIBRATE_COMMAND);
        }
    }


    @Override
    public void loop() {

        gyro1Cache = gyro1Synch.read(GYRO1_REG_START, GYRO1_READ_LENGTH);

        //One byte only goes to 256 values.  Two bytes must be combined to get 360 degrees
        heading = 0X000000FF & (int)gyro1Cache[1];
        heading = (heading << 8 | (0X000000FF & (int)gyro1Cache[0]));

        intZValue = 0X000000FF & (int)gyro1Cache[3];
        intZValue = (intZValue << 8 | (0X000000FF & (int)gyro1Cache[2]));

        // send the info back to driver station using telemetry function.

        telemetry.addData("Gyro Heading1", gyro1Cache[0]);
        telemetry.addData("Gyro Heading2", gyro1Cache[1]);
        telemetry.addData("Gyro HeadingTot", heading );
        telemetry.addData("Gyro Int Z1", gyro1Cache[2]);
        telemetry.addData("Gyro Int Z2", gyro1Cache[3]);
        telemetry.addData("Gyro Int ZTot", (short) intZValue);

    }


    @Override
    public void stop() {

    }
    /*
    I2C Registers
    Address            Function
    0x00            Sensor Firmware Revision
    0x01            Manufacturer Code
    0x02            Sensor ID Code
    0x03            Command
    0x04/0x05       Heading Data (lsb:msb)
    0x06/0x07       Integrated Z Value (lsb:msb)
    0x08/0x09       Raw X Value (lsb:msb)
    0x0A/0x0B       Raw Y Value (lsb:msb)
    0x0C/0x0D       Raw Z Value (lsb:msb)
    0x0E/0x0F       Z Axis Offset (lsb:msb)
    0x10/0x11       Z Axis Scaling Coefficient (lsb:msb)

    Commands
    Command	 Operation
    0x00	 Normal measurement mode
    0x4E	 Null gyro offset and reset Z axis integrator
    0x52	 Reset Z axis integrator
    0x57	 Write EEPROM data
*/
}