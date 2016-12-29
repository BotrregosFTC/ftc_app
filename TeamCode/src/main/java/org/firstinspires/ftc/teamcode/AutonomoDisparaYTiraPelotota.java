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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomo")
public class AutonomoDisparaYTiraPelotota extends LinearOpMode
{

    /* Declare OpMode members. */
    Hardwares hws           = new Hardwares();   // Use a Omni Drive Train's hardware

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declares variables used on the program
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        hws.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Dispara(1.0);
        Dispara(1.0);
        Dispara(1.0);
        StopDrivingTime();
        DriveForwardTime(DRIVE_POWER, 5000);
        StopDriving();

    }
    double DRIVE_POWER = .5;

    public void DriveForward (double power)
    {
        hws.frontLeftMotor.setPower(-power);
        hws.frontRightMotor.setPower(-power);
        hws.backLeftMotor.setPower(-power);
        hws.backRightMotor.setPower(-power);
    }
    public void TurnRight (double power)
    {
        hws.frontLeftMotor.setPower(power);
        hws.frontRightMotor.setPower(-power);
        hws.backLeftMotor.setPower(power);
        hws.backRightMotor.setPower(-power);
    }
    public void TurnLeft (double power)
    {
        hws.frontLeftMotor.setPower(-power);
        hws.frontRightMotor.setPower(power);
        hws.backLeftMotor.setPower(-power);
        hws.backRightMotor.setPower(power);
    }
    public void StopDriving ()
    {
        DriveForward(0);
    }
    public void StopDrivingTime(){
        DriveForward(0);
        sleep(4000);
    }

    public void StopDispara ()
    {
        Dispara(0);
    }

    public void DriveForwardTime (double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }
    public void TurnRightTime (double power, long time) throws InterruptedException
    {
        TurnRight(power);
        Thread.sleep(time);
    }
    public void TurnLeftTime (double power, long time) throws InterruptedException
    {
        TurnLeft(power);
        Thread.sleep(time);
    }
    public void Dispara (double power){
        hws.disparador.setPower(power);
    }

}

