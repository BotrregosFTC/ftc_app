/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the Mecanum hardware class to define the devices on the robot.
 * All device access is managed through the HardwareMecanumDriveTrain class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a MecanumDrive Bot.
 * This opmode uses the left stick to control the axial movement, while the right stick controls the rotation.

 */

@TeleOp(name="MecanumDriveTrain")
//@Disabled
public class MecanumDriveTrain extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwareMecanumWheels robotDrive           = new HardwareMecanumWheels();   // Use a Mecanum Drive Train's hardware
    HardwareColorSensor colorSensor = new HardwareColorSensor();
    HardwareServo servo = new HardwareServo ();
    HardwareOpticalDistanceSensor distanceSensor = new HardwareOpticalDistanceSensor();
    HardwareBrazo brazo = new HardwareBrazo();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
         /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robotDrive.init(hardwareMap);
        colorSensor.init(hardwareMap);
        servo.init(hardwareMap);
        distanceSensor.init(hardwareMap);
        brazo.init(hardwareMap);

        colorSensor.colorSensor.enableLed(colorSensor.bLedOn);

        // Send telemetry message to signify robot waiting;
        telemetry.addData ("Adry es la mejor del universo", "");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
           colorSensor.bCurrState = gamepad1.y;

            if ((colorSensor.bCurrState == true) && (colorSensor.bCurrState != colorSensor.bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                colorSensor.bLedOn = !colorSensor.bLedOn;
                colorSensor.colorSensor.enableLed(colorSensor.bLedOn);
            }

            // update previous state variable.
            colorSensor.bPrevState = colorSensor.bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.colorSensor.red() * 8, colorSensor.colorSensor.green() * 8, colorSensor.colorSensor.blue() * 8, colorSensor.hsvValues);

            telemetry.addData("LED", colorSensor.bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.colorSensor.red());
            telemetry.addData("Green", colorSensor.colorSensor.green());
            telemetry.addData("Blue ", colorSensor.colorSensor.blue());
            telemetry.addData("Hue", colorSensor.hsvValues[0]);

            telemetry.addData("Raw",    distanceSensor.odsSensor.getRawLightDetected());
            telemetry.addData("Normal", distanceSensor.odsSensor.getLightDetected());

            telemetry.update();


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            colorSensor.relativeLayout.post(new Runnable() {
                public void run() {

                    colorSensor.relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, colorSensor.values));
                }
            });

            if (gamepad1.a){
                servo.servo.setPosition(servo.Arm_Max);
            }
            else if (gamepad1.x){
                servo.servo.setPosition(servo.Arm_Min);
            }

            //Sets the turbo mode for the motors to normal when the right bumper is not pressed
            // or to max speed (turbo) when it is pressed
            if (gamepad1.right_bumper)
            {
                robotDrive.turbo = 1;
            }
            else
                robotDrive.turbo = .2;
            // Sets the joystick values to variables for better math understanding
            // The Y axis goes
            robotDrive.y1  = gamepad1.left_stick_y;
            robotDrive.x1  = gamepad1.left_stick_x;
            robotDrive.x2  = gamepad1.right_stick_x;

            // sets the math necessary to control the motors to variables
            // The left stick controls the axial movement
            // The right sick controls the rotation
            robotDrive.frontRightPower     = robotDrive.y1 - robotDrive.x2 - robotDrive.x1;
            robotDrive.backRightPower      = robotDrive.y1 - robotDrive.x2 + robotDrive.x1;
            robotDrive.frontLeftPower      = robotDrive.y1 + robotDrive.x2 + robotDrive.x1;
            robotDrive.backLeftPower       = robotDrive.y1 + robotDrive.x2 - robotDrive.x1;

            // Normalize the values so neither exceed +/- 1.0
            robotDrive.max =  Math.max(Math.abs(robotDrive.frontRightPower), Math.max(Math.abs(robotDrive.backRightPower),
                    Math.max(Math.abs(robotDrive.frontLeftPower), Math.abs(robotDrive.backLeftPower))));
            if (robotDrive.max > 1.0)
            {
                robotDrive.frontRightPower     /= robotDrive.max;
                robotDrive.backRightPower      /= robotDrive.max;
                robotDrive.frontLeftPower      /= robotDrive.max;
                robotDrive.backLeftPower       /= robotDrive.max;
            }

            // sets the speed for the motros with the turbo multiplier
            robotDrive.frontRightPower     *= robotDrive.turbo;
            robotDrive.backRightPower      *= robotDrive.turbo;
            robotDrive.frontLeftPower      *= robotDrive.turbo;
            robotDrive.backLeftPower       *= robotDrive.turbo;

            robotDrive.frontRightMotor.setPower(robotDrive.frontRightPower);
            robotDrive.backRightMotor.setPower(robotDrive.backRightPower);
            robotDrive.frontLeftMotor.setPower(robotDrive.frontLeftPower);
            robotDrive.backLeftMotor.setPower(robotDrive.backLeftPower);

            // Send telemetry message to signify robot running;
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robotDrive.waitForTick(40);
        }
    }
    public void DriveForwardDistance (double power, int distance){
        brazo.brazo.setMode(DcMotorController.RunMode.);


    }
}