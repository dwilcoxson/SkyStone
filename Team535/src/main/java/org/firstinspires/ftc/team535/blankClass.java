package org.firstinspires.ftc.team535;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

public class blankClass {
    public DcMotor left = null;
    public DcMotor right = null;

    HardwareMap map = null;

    public blankClass()
    {
    }

    public void initHW(HardwareMap passMap)
    {
        map = passMap;

        left = map.dcMotor.get("left");
        right = map.dcMotor.get("right");

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotor.Direction.REVERSE);
    }
}
