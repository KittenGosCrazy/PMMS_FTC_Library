package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class TeamDrivetrain {


    //Defines the motor variables (ex: frontLeft) as a DcMotor
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    public double frontLeftPower;
    public double backLeftPower;
    public double frontRightPower ;
    public double backRightPower;


    public IMU imu;

    private double radHeading;
    private double degHeading;


    //IMPORTANT FOR ENCODER CALCULATIONS
    //These are just placeholder variables until we can measure them
    //CPR = Counts Per Revolution
    //CPI = Counts Per Inch

    private static double gearRatio =22;
    private static double motorCPR = 28;
    private static double wheelDiameter = 3;

    //Adjusted Values in calculations
    private static double adjustedCPR = motorCPR*gearRatio;
    private static double driveCPI = adjustedCPR/(Math.PI * wheelDiameter);

    /*
    Call this at the start of your OpMode and it will Initialize the following:
    Drive Motors
    IMU/Gyro
     */
    public void initDrivetrain(HardwareMap hwMap) {

        //Drive Motor Initialization

        //Maps the Motors to the proper slot on the RevHubs. Name is found in the configuration menu
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        //Sets the right side to reverse
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Sets the motors to default as running using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

        //Prevents movement of motors, just in case.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //IMU (Gyroscope) initialization

        //Maps the IMU/Gyroscope to the correct spot in the configuration
        imu = hwMap.get(IMU.class,"imu");
        //Determines how the Rev Hub (with the IMU) is set up on your robot
        IMU.Parameters imuInit = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, //Change this to what direction your logo is facing
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT //Change this to what direction the usb port is facing
        )
        );
        //Tells the IMU/Gyroscope how the Control Hub (with the IMU) is set up using what we defined above
        imu.initialize(imuInit);
    }

    public void stopDrive(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    //Mecanum Driving
    public void robotDrive(double y, double x, double rot, double speedAdj) {

        //Math stuff for Mecanum Drive

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
        frontLeftPower = ((y + x + rot) / denominator)*speedAdj;
        backLeftPower = ((y - x + rot) / denominator)*speedAdj;
        frontRightPower = ((y - x - rot) / denominator)*speedAdj;
        backRightPower = ((y + x - rot) / denominator)*speedAdj;

        //Takes the results from the math and feeds it to the motor to get drive power
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


    }

    //Field Centric Driving
    public void fieldDrive(double y, double x, double rot, double speedAdj){

        //Gets the angle of the robot in radians
        //Used to determine relative rotation in relation to field
        double heading = getHeadingInRadians();

        //Takes the inputs we get and rotates them to make them in line with the field and not the robot
        double adjX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double adjY = x * Math.sin(-heading) + y * Math.cos(-heading);

        //Calls the Robot drive function, now using the updated inputs for field oriented
        robotDrive(adjY,adjX,rot,speedAdj);


    }

    //Returns gyro angle in respective unit

    public double getHeadingInRadians() {
        radHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return radHeading;
    }

    public double getHeadingInDegrees() {
        degHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return degHeading;
    }

    public double getDriveCPI() {
        return driveCPI;
    }
}
