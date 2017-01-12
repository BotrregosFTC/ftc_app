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
    Hardwares hws = new Hardwares ();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
         /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        hws.init(hardwareMap);

        double velDisp = 0.0;
        double positionS = 0.0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData ("Adry: Adry es la mejor del universo", "saul: no, no lo es");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.


            hws.disparador.setPower(gamepad2.right_trigger);
            telemetry.addData("disparando", "");

            if (gamepad2.right_trigger > .01)
            {
                velDisp = 1.0;

            }
            else
            {
                velDisp = 0.0;
            }

            telemetry.addData("disparador: ", velDisp) ;
            hws.disparador.setPower(velDisp);

            double banda_arriba = gamepad1.right_trigger;
            double banda_abajo = -gamepad1.left_trigger;
            double banda_direccion = 0;

            if(gamepad1.right_trigger > .01)
            {
                banda_direccion = banda_arriba;
            }

            else if(gamepad1.left_trigger > .1)
            {
                banda_direccion = banda_abajo;
            }
            else
            {
                banda_direccion=banda_direccion;
            }

            hws.elevador.setPower(banda_direccion);
            telemetry.addData("banda", banda_direccion);

            if (gamepad2.right_bumper)
            {
                positionS = .3;
            }
            else if (gamepad2.left_bumper)
            {
                positionS = 0.0;
            }

            telemetry.addData("servo: ", positionS) ;
            hws.servo.setPosition(positionS);

            //Sets the turbo mode for the motors to normal when the right bumper is not pressed
            // or to max speed (turbo) when it is pressed
            if (gamepad1.right_bumper)
            {
                hws.turbo = 1;
            }
            else
                hws.turbo = 1;
            // Sets the joystick values to variables for better math understanding
            // The Y axis goes
            hws.y1  = gamepad1.left_stick_y;
            hws.x1  = gamepad1.left_stick_x;
            hws.x2  = gamepad1.right_stick_x;

            // sets the math necessary to control the motors to variables
            // The left stick controls the axial movement
            // The right sick controls the rotation
            hws.frontRightPower     = hws.y1 + hws.x2 + hws.x1;
            hws.backRightPower      = hws.y1 + hws.x2 - hws.x1;
            hws.frontLeftPower      = hws.y1 - hws.x2 - hws.x1;
            hws.backLeftPower       = hws.y1 - hws.x2 +  hws.x1;

            // Normalize the values so neither exceed +/- 1.0
            hws.max =  Math.max(Math.abs(hws.frontRightPower), Math.max(Math.abs(hws.backRightPower),
                    Math.max(Math.abs(hws.frontLeftPower), Math.abs(hws.backLeftPower))));
            if (hws.max > 1.0)
            {
                hws.frontRightPower     /= hws.max;
                hws.backRightPower      /= hws.max;
                hws.frontLeftPower      /= hws.max;
                hws.backLeftPower       /= hws.max;
            }

            // sets the speed for the motros with the turbo multiplier
            hws.frontRightPower     *= hws.turbo;
            hws.backRightPower      *= hws.turbo;
            hws.frontLeftPower      *= hws.turbo;
            hws.backLeftPower       *= hws.turbo;

            hws.frontRightMotor.setPower(hws.frontRightPower);
            hws.backRightMotor.setPower(hws.backRightPower);
            hws.frontLeftMotor.setPower(hws.frontLeftPower);
            hws.backLeftMotor.setPower(hws.backLeftPower);

            // Send telemetry message to signify robot running;
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            hws.waitForTick(40);
        }
    }

}