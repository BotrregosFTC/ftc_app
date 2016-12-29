package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import java.util.IdentityHashMap;

/**
 * Created by saulo on 12/10/2016.
 */
@TeleOp(name="SaulAutonImageRecognition")
@Disabled
public class SaulAutonImageRecognition extends LinearOpMode {

    //Hardwares robot = new Hardwares ();

    @Override
    public void runOpMode() throws InterruptedException {

        //robot.init(hardwareMap);

        telemetry.addData ("Autonomous mode", "we can only hope it works");
        telemetry.update();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AWmM31n/////AAAAGQGLpAwyxEQRgx0+yduk09BKNJJgdFC4Ti1nfZAP8PSlO1C3uNWOPW/0eQefL19+S2VeUjx7Z3ZQGmYYLkqQfRiG2SB//RpH7+NIYK1TEkGG9pyiqYrlY4FJvkOOQ/bRdJSw7CjpRKaRqVUznO1srjlbu5HU3bDpgEYSzKJxmKHmcPMwAT5fmGFJHRw/KmMSmb4i9JpCd5nN9+rFTqRl2bFMnxjLqzCfwV+Dv8fiiZumZMLNEsvpi8fyAV1wRwz5vq44r595p28zsuA97E19hqe+KBlxOdomtIkZM9ThnUkaQZiH53WkuagKRigvRnhRBP4lX5W92nue5qfBp0Fnt5DEuW0oj6YEho4ulCK92sOu" ;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        beacons.get(0).setName("wheels");
        beacons.get(1).setName("tools");
        beacons.get(2).setName("lego");
        beacons.get(3).setName("gears");

        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
        VuforiaTrackableDefaultListener lego = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();
        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();



        double IntroSpeed = 1;
        double IdleSpeed = 0;
        double RotationSpeed = .2;
        double LowSpeed = .3;
        waitForStart();

        beacons.activate();

        //robot.backLeftMotor.setPower(IntroSpeed);
        //robot.backRightMotor.setPower(IntroSpeed);
        //robot.frontLeftMotor.setPower(IntroSpeed);
        //robot.frontRightMotor.setPower(IntroSpeed);
        telemetry.addData ("intro speed","");
        telemetry.update();

        while(opModeIsActive() && wheels.getRawPose()==null){
           idle();
        }

        //robot.backLeftMotor.setPower(IdleSpeed);
        //robot.backRightMotor.setPower(IdleSpeed);
        //robot.frontLeftMotor.setPower(IdleSpeed);
        //robot.frontRightMotor.setPower(IdleSpeed);
        telemetry.addData ("idle speed", "");
        telemetry.update();

        //analyze beacon

        VectorF angles = anglesFromTarget(wheels);

        VectorF trans = navOffWall(wheels.getPose().getTranslation() , Math.toDegrees(angles.get(0))-90, new VectorF(500,0,0));
        telemetry.addData ("translation",trans);
        telemetry.addData ("degrees", angles);
        telemetry.update();

        if(trans.get(0) > 0 ) {
           // robot.backLeftMotor.setPower(RotationSpeed);
           // robot.frontLeftMotor.setPower(RotationSpeed);
           // robot.backRightMotor.setPower(-RotationSpeed);
           // robot.frontRightMotor.setPower(-RotationSpeed);
            telemetry.addData ("translation",trans.get(0));
            telemetry.addData ("degrees", angles.get(0));
            telemetry.addData ("rotating right", "");
            telemetry.update();
        }else{
           // robot.backLeftMotor.setPower(-RotationSpeed);
           // robot.frontLeftMotor.setPower(-RotationSpeed);
           // robot.backRightMotor.setPower(RotationSpeed);.
           // robot.frontRightMotor.setPower(RotationSpeed);
            telemetry.addData ("translation",trans);
            telemetry.addData ("degrees", angles);
            telemetry.addData ("rotating left", "");
            telemetry.update();
        }

        do{
            if(wheels.getPose() !=null){
               trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0))-90, new VectorF(500, 0, 0));
            }
            idle();
        }while (opModeIsActive()&& Math.abs(trans.get(0))>30);
        // robot.backLeftMotor.setPower(IdleSpeed);
        // robot.frontLeftMotor.setPower(IdleSpeed);
        // robot.backRightMotor.setPower(IdleSpeed);
        // robot.frontRightMotor.setPower(IdleSpeed);
        telemetry.addData ("translation",trans);
        telemetry.addData ("degrees", angles);
        telemetry.addData ("idle speed", "");
        telemetry.update();


        while(trans.get(1) > 0){

           // robot.backLeftMotor.setPower(LowSpeed);
           // robot.frontLeftMotor.setPower(LowSpeed);
           // robot.backRightMotor.setPower(LowSpeed);
           // robot.frontRightMotor.setPower(LowSpeed);
            telemetry.addData ("translation",trans);
            telemetry.addData ("degrees", angles);
            telemetry.addData ("low speed", "");
            telemetry.update();

        }

        while (opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0))>10)){
            if (wheels.getPose() !=null) {
                if(wheels.getPose().getTranslation().get(0)>0){
                   // robot.backLeftMotor.setPower(RotationSpeed);
                   // robot.frontLeftMotor.setPower(RotationSpeed);
                   // robot.backRightMotor.setPower(-RotationSpeed);
                   // robot.frontRightMotor.setPower(-RotationSpeed);
                    telemetry.addData ("translation",trans);
                    telemetry.addData ("degrees", angles);
                    telemetry.addData ("rotating right", "");
                    telemetry.update();
                }else{
                   // robot.backLeftMotor.setPower(-RotationSpeed);
                   // robot.frontLeftMotor.setPower(-RotationSpeed);
                   // robot.backRightMotor.setPower(RotationSpeed);
                   // robot.frontRightMotor.setPower(RotationSpeed);
                    telemetry.addData ("translation",trans);
                    telemetry.addData ("degrees", angles);
                    telemetry.addData ("rotating left", "");
                    telemetry.update();
                }
            }else {
               // robot.backLeftMotor.setPower(RotationSpeed);
               // robot.frontLeftMotor.setPower(RotationSpeed);
               // robot.backRightMotor.setPower(-RotationSpeed);
               // robot.frontRightMotor.setPower(-RotationSpeed);
                telemetry.addData ("translation",trans);
                telemetry.addData ("degrees", angles);
                telemetry.addData ("rotating right", "");
                telemetry.update();
            }
        }
       // robot.backLeftMotor.setPower(IdleSpeed);
       // robot.frontLeftMotor.setPower(IdleSpeed);
       // robot.backRightMotor.setPower(IdleSpeed);
       // robot.frontRightMotor.setPower(IdleSpeed);
        telemetry.addData ("translation",trans);
        telemetry.addData ("degrees", angles);
        telemetry.addData ("idle speed", "");
        telemetry.update();




    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){ return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){ float [] data = image.getRawPose().getData(); float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}}; double thetaX = Math.atan2(rotation[2][1], rotation[2][2]); double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2])); double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]); return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }
}
