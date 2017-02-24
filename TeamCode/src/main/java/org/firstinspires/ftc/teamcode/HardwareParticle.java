package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
public class HardwareParticle
{
    /* Public OpMode members. */
    public DcMotor  particlePickUp   = null;
    public DcMotor  shooter  = null;
    public Servo particleLift = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareParticle()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        particlePickUp   = hwMap.dcMotor.get("pickup");
        shooter  = hwMap.dcMotor.get("shooter");
        particleLift = hwMap.servo.get("spl");


        particlePickUp.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        particlePickUp.setPower(0);
        shooter.setPower(0);
        particleLift.setPosition(0.0);

        // Set all motors to run without encoders.
        particlePickUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void pickUp(int direction,double power)
    {
        particlePickUp.setPower(direction*power);
    }
    public void shooting()
    {
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setTargetPosition(1120);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(.7);
    }
    public void partLift()
    {
        particleLift.setPosition(.3);
    }
}
