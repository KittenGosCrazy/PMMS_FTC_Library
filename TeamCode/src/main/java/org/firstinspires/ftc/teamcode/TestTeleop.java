package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Test: MainTeleop", group="Linear Opmode")
public class TestTeleop extends LinearOpMode {

    //Create the drivetrain object (Allows us to call it here)
    TeamDrivetrain drivetrain = new TeamDrivetrain();

    @Override
    public void runOpMode() {

        //Sets the hardware map for the motors. Robot now knows what port the motors are plugged in to
        drivetrain.initDrivetrain(hardwareMap);
        double speedAdjustment = 1;

        waitForStart();

        while(opModeIsActive()) {

            //Creates Variables for Gamepad inputs
            double yInput = -gamepad1.left_stick_y;
            double xInput = gamepad1.left_stick_x * 1.1;
            double rotInput = gamepad1.right_stick_x;


            if (gamepad1.right_trigger < 0.5) speedAdjustment = 1;
            else if (gamepad1.right_trigger > 0.5) speedAdjustment = 0.6;
            drivetrain.fieldDrive(xInput, yInput, rotInput, speedAdjustment);

            telemetry.update();

        }
    }
}