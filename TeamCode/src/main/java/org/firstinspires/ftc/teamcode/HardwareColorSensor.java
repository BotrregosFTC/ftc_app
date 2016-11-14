package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareColorSensor
{
    /* Public OpMode members. */
    HardwareMap hwMap           =  null;

    public ColorSensor colorSensor;
    public float hsvValues[] = {0F,0F,0F};
    public final float values[] = hsvValues;
    public final View relativeLayout = ((Activity) hwMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    public boolean bPrevState = false;
    public boolean bCurrState = false;
    public boolean bLedOn = true;

    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareColorSensor()
    {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {

        // Save reference to Hardware map
        hwMap = ahwMap;

        colorSensor = hwMap.colorSensor.get("sensor_color");
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
