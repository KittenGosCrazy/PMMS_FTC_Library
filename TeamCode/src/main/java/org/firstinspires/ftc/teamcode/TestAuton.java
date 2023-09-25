package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "Test Autonomous", group="Linear Opmode")
public class TestAuton extends LinearOpMode {

    TeamDrivetrain drivetrain = new TeamDrivetrain();

    @Override
    public void runOpMode() {

        drivetrain.initDrivetrain(hardwareMap);
        int state = 0;

        waitForStart();

        while (opModeIsActive()) {
            //Drive forward and turn 90 degrees to the right
            if (state == 0) {
                if(drivetrain.autoDrive(12, 1, 90, 0, true)) state = 10;
            }
            //Drive to the left, and turn to a rotation of 45 degrees
            if (state == 10) {
                if(drivetrain.autoDrive(6, 0.5, 45, 270, true)) state = 999;
            }
            //Stop Driving
            if (state == 999) {
                drivetrain.stopDrive();
            }
        }
    }
}
