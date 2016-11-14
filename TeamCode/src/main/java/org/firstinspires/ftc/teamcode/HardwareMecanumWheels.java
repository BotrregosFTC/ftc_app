package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class declares and initializes all hardware needed to control a Mecanum Drive Train
 * Motor channel:  Front Right Motor:        "fr"
 * Motor channel:  Back Right Motor:        "br"
 * Motor channel:  Front Left Motor:        "fl"
 * Motor channel:  Back Left Motor:        "bl"
 *
 */
public class HardwareMecanumWheels
{
    /* Public OpMode members. */
    public DcMotor  frontRightMotor   = null;
    public DcMotor  backRightMotor  = null;
    public DcMotor  frontLeftMotor    = null;
    public DcMotor  backLeftMotor    = null;
    public double y1;
    public double x1;
    public double x2;
    public double frontRightPower;
    public double backRightPower;
    public double frontLeftPower;
    public double backLeftPower;
    public double max;
    public double turbo;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanumWheels()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRightMotor   = hwMap.dcMotor.get("fr");
        backRightMotor  = hwMap.dcMotor.get("br");
        frontLeftMotor   = hwMap.dcMotor.get("fl");
        backLeftMotor  = hwMap.dcMotor.get("bl");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE to normalize movement
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE to normalize movement

        // Set all motors to zero power
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);

        // Set all motors to run without encoders.
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException
    {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
