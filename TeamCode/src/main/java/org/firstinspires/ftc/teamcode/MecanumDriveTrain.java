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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    HardwareMecanumDriveTrain robotDrive           = new HardwareMecanumDriveTrain();   // Use a Mecanum Drive Train's hardware

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
        double y1;
        double x1;
        double x2;
        double frontRightPower;
        double backRightPower;
        double frontLeftPower;
        double backLeftPower;
        double max;
        double turbo;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robotDrive.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("here comes dat bot", "Oh hey, waddup");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            //Sets the turbo mode for the motors to normal when the right bumper is not pressed
            // or to max speed (turbo) when it is pressed
            if (gamepad1.right_bumper)
            {
                turbo = 1;
            }
            else
                turbo = .2;
            // Sets the joystick values to variables for better math understanding
            // The Y axis goes
            y1  = gamepad1.left_stick_y;
            x1  = gamepad1.left_stick_x;
            x2  = gamepad1.right_stick_x;

            // sets the math necessary to control the motors to variables
            // The left stick controls the axial movement
            // The right sick controls the rotation
            frontRightPower     = y1 - x2 - x1;
            backRightPower      = y1 - x2 + x1;
            frontLeftPower      = y1 + x2 + x1;
            backLeftPower       = y1 + x2 - x1;

            // Normalize the values so neither exceed +/- 1.0
            max =  Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backRightPower),
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower))));
            if (max > 1.0)
            {
                frontRightPower     /= max;
                backRightPower      /= max;
                frontLeftPower      /= max;
                backLeftPower       /= max;
            }

            // sets the speed for the motros with the turbo multiplier
            frontRightPower     *= turbo;
            backRightPower      *= turbo;
            frontLeftPower      *= turbo;
            backLeftPower       *= turbo;

            robotDrive.frontRightMotor.setPower(frontRightPower);
            robotDrive.backRightMotor.setPower(backRightPower);
            robotDrive.frontLeftMotor.setPower(frontLeftPower);
            robotDrive.backLeftMotor.setPower(backLeftPower);

            // Send telemetry message to signify robot running;
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robotDrive.waitForTick(40);
        }
    }
}