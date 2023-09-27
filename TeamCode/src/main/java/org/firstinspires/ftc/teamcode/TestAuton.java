package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "Test Autonomous", group="Linear Opmode")
public class TestAuton extends LinearOpMode {

    AutoDrivetrain drivetrain = new AutoDrivetrain(0,0); //Offset X, Offset Y

    @Override
    public void runOpMode() {

        drivetrain.initDrivetrain(hardwareMap);
        int state = 0;

        waitForStart();

        while (opModeIsActive()) {

            //Records changes in position
            drivetrain.recordPose();

            if (state == 0) {
                if (drivetrain.autoDriveToDistance(12,0.5,90,15,true)) state = 10;
            }
            if (state == 10) {
                if (drivetrain.autoDriveToPosition(0,0,0.5, 0)) state = 999;
            }
            //Stop Driving
            if (state == 999) {
                drivetrain.stopDrive();
            }

            //TELEMETRY
            telemetry.addData("State", state);
            telemetry.addData("Position X", drivetrain.getPoseX());
            telemetry.addData("Position Y", drivetrain.getPoseY());
            telemetry.addData("Target X", drivetrain.getTargetX());
            telemetry.addData("Target Y", drivetrain.getTargetY());
            telemetry.addData("Auto Counter", drivetrain.getAutoCounter());
        }

    }
}
