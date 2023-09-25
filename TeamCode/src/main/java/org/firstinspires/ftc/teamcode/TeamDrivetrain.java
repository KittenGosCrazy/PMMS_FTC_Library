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

    private static double gearRatio = 5.23;
    private static double motorCPR = 28;
    private static double wheelDiameter = 1;

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

        //Updates Telemetry
        telemetry.addData("Front Left Power", this.frontLeftPower);
        telemetry.addData("Back Left Power", this.backLeftPower );
        telemetry.addData("Front Right Power", this.frontRightPower);
        telemetry.addData("Back Right Power", this.backRightPower);

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

        //Telemetry
        telemetry.addData("Heading in Radians", this.getHeadingInRadians());
        telemetry.addData("Heading in Degrees", this.getHeadingInDegrees());

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

    public double getDriveDistance(){
        double yDistance = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition())/4;
        double xDistance = (((frontLeft.getCurrentPosition()-backRight.getCurrentPosition())+(frontRight.getCurrentPosition()-backLeft.getCurrentPosition()))/4);
        return yDistance + xDistance;
    }

    //Returns false if motors are not at target
    //Returns true if motors are at target
    //ONLY WORKS IN RUN TO POSITION
    private boolean motorsAtTarget() {
        if(frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()) return false;
        else return true;
    }



    //AUTONOMOUS DRIVE FUNCTIONS

    //Variables needed initialization outside of a loop
    private boolean autoDriveRunning = false;
    private double initialPosition;
    private double initialRotation;

    //Telemetry Variables
    private double distanceRemaining;
    private double rotationRemaining;

    //Defines Autonomous Constants for adjustment
    private double kMecanumDrift = 1; //To get: set this to 1, then run it to a distance. Measure the actual distance and divide it by desired distance
    private final double kRotFactor = 1; //Multiplies by the poseFactor to adjust it's weight
    private final double kSlipFactor = 0.25; //Deadzone adjustment for driving


    //Returns a false if still running
    //Returns a true if it has stopped running
    //distance in inches to move, desiredSpeed (1-0), desiredRotation to rotate the robot,
    //travelHeading for the heading it travels, fieldCentric for whether it is field centric or not
    public boolean autoDrive(double distance, double desiredSpeed, double desiredRotation, double travelHeading, boolean fieldCentric) {

        //Sets the boolean for if we are at targets
        boolean atRotation = false; //Is rotation complete?
        boolean atDistance = false; //Is distance complete?
        boolean atTarget = false; //Robot at both targeted positions? (Stopped running)


        //Set Current, Initial, and Targeted Positions
        double currentRotation = getHeadingInDegrees(); //Records Rotation
        double currentPosition = getDriveDistance()/driveCPI; //Records Distance
        //On first run of this call:
        if(!autoDriveRunning) {
            initialPosition = currentPosition; //Start Spot
            initialRotation = currentRotation; //Start Rotation
            autoDriveRunning = true; //Makes sure robot knows this is running
        }

        double targetedPosition = (initialPosition + distance) * kMecanumDrift; //Adjusts the position relative to the start position of the robot

        double poseRatio; //Ratio between rotation difference and position difference
        //Make sure its not dividing by 0, and then get a ratio that's always positive
        if (targetedPosition - initialPosition != 0) poseRatio = Math.abs((desiredRotation-initialRotation)/(targetedPosition-initialPosition));
        else poseRatio = 1;

        //INPUT VARIABLES
        double inputX; //Movement side-to-side
        double inputY; //Movement forward/back
        double inputRot; //Rotational Movement


        //DISTANCE CALCULATIONS
        //Sets the Input X and Ys after checking where the robot is in relation to it's target. Deadzones set by the "Slip Factor"
        if (currentPosition > targetedPosition + kSlipFactor || currentPosition < targetedPosition - kSlipFactor) {
            //Gets ratio between X-Y
            inputX = Math.sin(travelHeading);
            inputY = Math.cos(travelHeading);
        }
        else {
            //ONLY RUNS WHEN WE ARE AT CORRECT DISTANCE
            inputX = 0;
            inputY = 0;
            atDistance = true;
        }


        //ROTATION CALCULATIONS
        //Sets rotation speed according to poseRatio. Multiplies by kRotFactor for better control and adjustment
        if(currentRotation > desiredRotation + 1){
            inputRot = poseRatio * kRotFactor;
        }
        else if (currentRotation < desiredRotation - 1) {
            inputRot = - poseRatio * kRotFactor;
        }
        else {
            //ONLY RUNS WHEN WE ARE CORRECT ROTATION
            atRotation = true;
            inputRot = 0;
        }


        //DRIVE
        //Call the Drive function in accordance with the "fieldCentric" boolean
        if (fieldCentric == true) fieldDrive(inputY,inputX,inputRot,desiredSpeed); //Calls field centric when method calls for it
        else robotDrive(inputY,inputX,inputRot,desiredSpeed); //Calls robot centric when method doesn't want field centric

        //TELEMETRY
        telemetry.addData("Auto Drive Running", this.autoDriveRunning);

        distanceRemaining = currentPosition-initialPosition;
        telemetry.addData("Inches to Target Distance", this.distanceRemaining);

        rotationRemaining = currentRotation-initialRotation;
        telemetry.addData("Degrees to Target Rotation", this.rotationRemaining);


        //If we are at the target position (both rot and distance), reset for next call
        if(atRotation && atDistance) atTarget = true;
        if(atTarget == true) autoDriveRunning = false;

        //Returns whether we are at target
        return atTarget;

    }

}
