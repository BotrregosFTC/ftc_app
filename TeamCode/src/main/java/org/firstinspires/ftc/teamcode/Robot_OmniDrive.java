package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left drive"
 * Motor channel:  Right drive motor:        "right drive"
 * Motor channel:  Rear  drive motor:        "back drive"
 *
 * These motors correspond to three drive locations spaced 120 degrees around a circular robot.
 * Each motor is attached to an omni-wheel. Two wheels are in front, and one is at the rear of the robot.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_OmniDrive
{
    // Private Members
    private LinearOpMode myOpMode;

    public DcMotor  frontLeftDrive      = null;
    public DcMotor  frontRightDrive    = null;
    public DcMotor  backLeftDrive      = null;
    public DcMotor  backRightDrive     = null;

    public double  driveAxial      = 0 ;   // Positive is forward
    public double  driveLateral    = 0 ;   // Positive is right
    public double  driveYaw        = 0 ;   // Positive is CCW

    double frontRightPower;
    double backRightPower;
    double frontLeftPower;
    double backLeftPower;

    /* Constructor */
    public Robot_OmniDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {

        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        frontLeftDrive           = myOpMode.hardwareMap.get(DcMotor.class, "fl");
        frontRightDrive         = myOpMode.hardwareMap.get(DcMotor.class, "fr");
        backLeftDrive           = myOpMode.hardwareMap.get(DcMotor.class, "bl");
        backRightDrive          = myOpMode.hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //use RUN_USING_ENCODERS because encoders are installed.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
    }

    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(myOpMode.gamepad1.right_stick_x);
    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot()
    {
        // calculate required motor speeds to acheive axis motions

        frontRightPower     = -driveAxial - driveLateral - driveYaw;
        backRightPower      = -driveAxial - driveLateral + driveYaw;
        frontLeftPower      = -driveAxial + driveLateral + driveYaw;
        backLeftPower       = -driveAxial + driveLateral - driveYaw;

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0)
        {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Set drive motor power levels.
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);

    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }
}

