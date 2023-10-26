package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name= "Test: ExampleAuton", group="Linear Opmode")
public class ExampleAuto extends LinearOpMode {

    AutoTrajectoryHelper traverseField = new AutoTrajectoryHelper();

    @Override
    public void runOpMode(){
        traverseField.setStart(0,0,0);
        traverseField.setEnd(120,120,0);

    }
}
