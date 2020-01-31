package org.firstinspires.ftc.team535;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ethan Blue", group="TeleOp")
//@Disabled
public class arcadeDriveEthanBlue extends OpMode
{
    hardwareTOBOR robo = new hardwareTOBOR();
    boolean brakeToggle = true;
    double speedControl = 1;
    boolean isBrake = true;
    final double hookDownPos = 0.0;
    final double hookUpPos = 1.0;
    boolean buttonPushed = false;
    @Override
    public void init() {
        robo.initHW(hardwareMap);

    }

    @Override
    public void init_loop() {
        robo.armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        robo.resetZAxis(true,false);
    }

    @Override
    public void start() {
        robo.directionLock = robo.getIntegratedZAxis();
        robo.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        /*
        GAMEPAD 1
        LEFT STICK: ARCADE TRANSLATION
        RIGHT STICK: ARCADE ROTATION
        RIGHT STICK BUTTON + LEFT STICK BUTTON: TOGGLE BRAKING

        DPAD UP: SPEED CONTROL FULL
        DPAD LEFT/DPAD RIGHT: SPEED CONTROL MID
        DPAD DOWN: SPEED CONTROL LOW

        A/B/X/Y: PRESET GYRO NUMBERS

        RIGHT BUMPER: T-REX ARMS OUT
        LEFT BUMPER: T-REX ARMS IN

        BACK: RESETS ARCADE DRIVE TO CURRENT GYRO



        --------------------
        GAMEPAD2
        LEFT STICK: ELBOW
        RIGHT STICK: ARM EXTENSION

        RIGHT BUMPER: SLED OUT
        LEFT BUMPER: SLED IN
        RIGHT TRIGGER: CLOSE SLURP
        LEFT TRIGGER: OPEN SLURP

         */
        if (gamepad1.left_trigger>=0.1)
        {
            robo.hooksGrab(2);
        }
        else if (gamepad1.right_trigger>=0.1)
        {
            robo.hooksGrab(1);
        }

        if (gamepad2.right_bumper)
        {
            robo.SLED.setPower(1);
        }
        else if (gamepad2.left_bumper)
        {
            robo.SLED.setPower(-1);
        }
        else
        {
            robo.SLED.setPower(0);

        }
        robo.elbow.setPower(gamepad2.left_stick_y*0.6);
        if (gamepad2.right_trigger>0.1)
        {
            robo.slurp(false);
        }
        else if (gamepad2.left_trigger>0.1)
        {
            robo.slurp(true);
        }
        telemetry.addData("SLURP Pos",robo.SLURP.getPosition());
        if (gamepad1.back)
        {
            robo.stopMotors();
            robo.resetZAxis(false,false);
            robo.directionLock = robo.getIntegratedZAxis();
        }
        else
        {
            robo.arcadeDrive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,speedControl);
        }

        if (gamepad1.dpad_down)
        {
            speedControl = 0.66;
        }
        else if (gamepad1.dpad_right||gamepad1.dpad_left)
        {
            speedControl = 0.89;
        }
        else if (gamepad1.dpad_up)
        {
            speedControl = 1;
        }

        if (gamepad1.right_stick_button&&gamepad1.left_stick_button&&brakeToggle)
        {
            isBrake = !isBrake;
            brakeToggle = false;
        }

        if (!(gamepad1.right_stick_button&&gamepad1.left_stick_button))
        {
            brakeToggle = true;
        }

        if (isBrake)
        {
            robo.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robo.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            robo.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robo.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        double zAxis = robo.getIntegratedZAxis();
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
        if (!buttonPushed)
        {
            if (gamepad1.y)
            {
                double upLevel = 360-currentHeading;
                double downLevel = currentHeading;
                if (currentHeading<=180)
                {
                    robo.directionLock -= downLevel;
                }
                else
                {
                    robo.directionLock += upLevel;
                }
                buttonPushed = true;
            }
            else if (gamepad1.x)
            {
                double upLevel = currentHeading<=90 ? 90-currentHeading : 450-currentHeading;
                double downLevel = currentHeading-90;
                if (currentHeading>=90&&currentHeading <=270)
                {
                    robo.directionLock -= downLevel;
                }
                else
                {
                    robo.directionLock += upLevel;
                }
                buttonPushed = true;
            }
            else if (gamepad1.a)
            {
                double upLevel = 180-currentHeading;
                double downLevel = currentHeading-180;
                if (currentHeading>=180)
                {
                    robo.directionLock -= downLevel;
                }
                else
                {
                    robo.directionLock += upLevel;
                }
                buttonPushed = true;
            }
            else if (gamepad1.b)
            {
                double upLevel = 270-currentHeading;
                double downLevel = currentHeading<=90 ? currentHeading+90 : currentHeading-270;
                if (currentHeading>=270||currentHeading<=90)
                {
                    robo.directionLock -= downLevel;
                }
                else
                {
                    robo.directionLock += upLevel;
                }
                buttonPushed = true;
            }
            else
            {
                buttonPushed = false;
            }
        }
        else if (!(gamepad1.y||gamepad1.x||gamepad1.a||gamepad1.b))
        {
            buttonPushed = false;
        }

        robo.armExtend.setPower(Range.clip(-gamepad2.right_stick_y,-1,1));

        telemetry.addData("Front Left Pos", robo.frontLeft.getCurrentPosition());
        telemetry.addData("Back Left Pos", robo.backLeft.getCurrentPosition());
        telemetry.addData("Front Right Pos", robo.frontRight.getCurrentPosition());
        telemetry.addData("Back Right Pos", robo.backRight.getCurrentPosition());
        telemetry.addData("","");
        telemetry.addData("","");
        telemetry.addData("","");
        telemetry.addData("Speed Scalar", speedControl);
        telemetry.addData("Direction Lock", robo.directionLock);
        telemetry.addData("angle", robo.angles.firstAngle);
        telemetry.addData("Z axis", robo.getIntegratedZAxis());
    }

    @Override
    public void stop() {
    }
}
