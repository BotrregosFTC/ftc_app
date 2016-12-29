package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import com.vuforia.VuforiaBase;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import java.util.LinkedHashMap;

/**
 * Created by saulo on 12/10/2016.
 */
@TeleOp(name="vuforia tutorial")
@Disabled
public class vuforiaOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

        beacons.activate();
        while(opModeIsActive()){
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beac.getListener()).getPose();

                if(pose != null) {
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-translation",translation);
                    double degreesToTurn = Math.toDegrees((Math.atan2(translation.get(0),translation.get(1))));
                    telemetry.addData(beac.getName() + "-degrees", degreesToTurn);

                }
            }
            telemetry.update();
        }
        

    }
}
