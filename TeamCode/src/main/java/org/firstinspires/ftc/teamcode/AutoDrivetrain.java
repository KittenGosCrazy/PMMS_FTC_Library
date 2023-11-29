package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoDrivetrain extends TeamDrivetrain {


    //Position/Odometry Variables
    private double poseX;
    private double poseY;

    private double lastX;
    private double lastY;
    private double lastRot = 0;

    private double deltaRobotX; //Change in Robot Relative (RR) X
    private double deltaRobotY; //Change in Robot Relative (RR) Y

    private double deltaFieldX; //Change in Field Relative (FR) X
    private double deltaFieldY; //Change in Field Relative (FR) Y

    private double targetX;
    private double targetY;

    //Variables needed initialization outside of a loop
    private boolean autoDriveRunning = false;
    private double initialXPosition;
    private double initialYPosition;
    private double initialRotation;

    //Telemetry Variables
    private double autoCounter;
    private double rotationRemaining;


    //Defines Autonomous Constants for adjustment
    private double kMecanumDrift = 1; //To get: set this to 1, then run it to a distance. Measure the actual distance and divide it by desired distance
    private final double kRotFactor = .15; //Multiplies by the poseFactor to adjust it's weight
    private final double kSlipFactor = 2.5; //Deadzone adjustment for driving
    private final double kRotDeadzone = 5;

    Translation2d flWheelLocation =
            new Translation2d(0.381, 0.381);
    Translation2d frWheelLocation =
            new Translation2d(0.381, -0.381);
    Translation2d blWheelLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d brWheelLocation =
            new Translation2d(-0.381, -0.381);

    MecanumDriveOdometry odometryHelper;
    MecanumDriveKinematics wheelLocationHelper;

    ElapsedTime autoTimer = new ElapsedTime();
    ElapsedTime veloTimer = new ElapsedTime();
    /*
  --------------------------
---------CONSTRUCTORS---------
  --------------------------

    ----------------------
    --NO-ARG CONSTRUCTOR--
    ----------------------
    Sets Pose X and Pose Y to 0

    ----------------------
    ---ARG CONSTRUCTORS---
    ----------------------
    double startX, double startY

    Sets the start position (pose) of the robot

    */



    public AutoDrivetrain() {
        wheelLocationHelper = new MecanumDriveKinematics(
                flWheelLocation, frWheelLocation,
                blWheelLocation, brWheelLocation
        );

        odometryHelper = new MecanumDriveOdometry(
                wheelLocationHelper, new Rotation2d(getHeadingInRadians()),
                new Pose2d(0,0, new Rotation2d())
        );
    }

    public AutoDrivetrain(double startX, double startY) {
        wheelLocationHelper = new MecanumDriveKinematics(
                flWheelLocation, frWheelLocation,
                blWheelLocation, brWheelLocation
        );

        odometryHelper = new MecanumDriveOdometry(
                wheelLocationHelper, Rotation2d.fromDegrees(getHeadingInDegrees()),
                new Pose2d(startX,startY, new Rotation2d())
        );
    }


    public void startTimer() {
        autoTimer.startTime();
        veloTimer.startTime();
    }

    //Records Current Pose
    public void recordPose() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                getFLVelo(), getFRVelo(),
                getBLVelo(), getBRVelo()
        );
        veloTimer.reset();

        Rotation2d gyroAngle = Rotation2d.fromDegrees(getHeadingInDegrees());

        Pose2d overallPose = odometryHelper.updateWithTime(autoTimer.time(), gyroAngle, wheelSpeeds);

        poseX = overallPose.getX();
        poseY = overallPose.getY();

        autoCounter++;
    }

    //Returns a false if still running
    //Returns a true if it has stopped running
    //distance in inches to move, desiredSpeed (1-0), desiredRotation to rotate the robot,
    //travelHeading for the heading it travels, fieldCentric for whether it is field centric or not
    public boolean autoDriveToDistance(double distance, double desiredSpeed, double desiredRotation, double travelHeading, boolean fieldCentric) {

        //Sets the boolean for if we are at targets
        boolean atRotation = false; //Is rotation complete?

        boolean atX = false; //Is it at targetX?
        boolean atY = false; //Is it at targetY?
        boolean atDistance = false; //Is it both at tX and tY?

        boolean atTarget = false; //Robot at both targeted positions? (Stopped running)

        double travelHeadingInRadians = Math.toRadians(travelHeading);

        //Set Current, Initial, and Targeted Positions
        double currentRotation = getHeadingInDegrees(); //Records Rotation

        //On first run of this call:
        if (!autoDriveRunning) {
            initialXPosition = poseX; //Start X Pose when first called
            initialYPosition = poseY; //Start Y pose when first called
            initialRotation = currentRotation; //Start Rotation
            autoDriveRunning = true; //Makes sure robot knows this is running
            autoCounter++;  //Counts the amount of 'First Calls' this has had
        }

        targetX = ((Math.sin(travelHeadingInRadians) * distance) * kMecanumDrift) + initialXPosition; //Sets the targeted X position on the field
        targetY = ((Math.cos(travelHeadingInRadians) * distance) * kMecanumDrift) + initialYPosition; //Sets the targeted Y position on the field

        double poseRatio; //Ratio between rotation difference and position difference
        if (distance != 0)
            poseRatio = Math.abs((desiredRotation - initialRotation) / (distance));  //Make sure its not dividing by 0
        else poseRatio = 1; //In case it divides by 0, its set to 1

        //INPUT VARIABLES
        double inputX; //Movement side-to-side
        double inputY; //Movement forward/back
        double inputRot; //Rotational Movement


        //DISTANCE CALCULATIONS
        //Sets the Input X and Ys after checking where the robot is in relation to it's targets. Deadzones set by the "Slip Factor"

        //Checks for if poseX is at targetX. if it isn't assign inputX after math
        if (poseX > (targetX + kSlipFactor * 2) * kMecanumDrift || poseX < (targetX - kSlipFactor * 2) * kMecanumDrift)
            inputX = Math.sin(travelHeadingInRadians);
        else if (poseX > (targetX + kSlipFactor) * kMecanumDrift || poseX < targetX - (targetX - kSlipFactor) * kMecanumDrift) {
            inputX = Math.sin(travelHeadingInRadians);
            desiredSpeed = desiredSpeed / 1.5;
        }
        else { //don't send inputX and flag it as atX
            inputX = 0;
            atX = true;
        }

        //Checks for if poseY is at targetY. if it isn't assign inputY after math
        if (poseY > targetY + kSlipFactor * 2 || poseY < targetY - kSlipFactor * 2)
            inputY = Math.cos(travelHeadingInRadians);
        else if (poseY > targetY + kSlipFactor || poseY < targetY - kSlipFactor) {
            inputY = Math.cos(travelHeadingInRadians);
            desiredSpeed = desiredSpeed / 1.5;
        }
        else { //don't send inputY and flag it as atY
            inputY = 0;
            atY = true;
        }

        if (atY && atX) atDistance = true;//If we are at X and at Y, flag that we are at distance


        //ROTATION CALCULATIONS
        //Sets rotation speed according to poseRatio. Multiplies by kRotFactor for better control and adjustment
        if (currentRotation > -desiredRotation + kRotDeadzone) {
            inputRot = poseRatio * kRotFactor;
        }
        else if (currentRotation < -desiredRotation - kRotDeadzone) {
            inputRot = -poseRatio * kRotFactor;
        }
        else {
            //ONLY RUNS WHEN WE ARE CORRECT ROTATION
            atRotation = true;
            inputRot = 0;
        }


        //DRIVE
        //Call the Drive function in accordance with the "fieldCentric" boolean
        if (fieldCentric == true)
            fieldDrive(inputY, inputX, inputRot, desiredSpeed); //Calls field centric when method calls for it
        else
            robotDrive(inputY, inputX, inputRot, desiredSpeed); //Calls robot centric when method doesn't want field centric


        //Telemetry Adjustments
        rotationRemaining = currentRotation - initialRotation;


        //If we are at the target position (both rot and distance), reset for next call
        if (atRotation && atDistance) atTarget = true;
        if (atTarget == true) autoDriveRunning = false;

        //Returns whether we are at target
        return atTarget;

    }

    //Returns a false if still running
    //Returns a true if it has stopped running
    //Give it a targetX and targetY on the field, and it drives to it
    //Speed range from 0 to 1
    //desiredRotation to set where the robot's heading will be once done
    public boolean autoDriveToPosition(double targetX, double targetY, double desiredSpeed, double desiredRotation) {

        //On first run of this call:
        if (!autoDriveRunning) {
            initialXPosition = poseX; //Start X Pose when first called
            initialYPosition = poseY; //Start Y pose when first called
        }

        double travelX = targetX - initialXPosition; //Distance Traveled on the X axis
        double travelY = targetY - initialYPosition; //Distance Traveled on the Y axis
        double travelHeading = Math.atan(travelX / travelY); //Theta (angle) of travel
        double distance = travelX / Math.sin(travelHeading); //Total distance traveled

        travelHeading = Math.toDegrees(travelHeading);


        return autoDriveToDistance(distance, desiredSpeed, desiredRotation, travelHeading, true);
    }



    /*

    --------------------------------
    --------ACCESSOR METHODS--------
    --------------------------------

    --1: General Telemetry Accessors

    --2: Odometry Specific Accessors
    ---2a: Raw Counts to Inches
    ---2b: Current Position
    ---2c: Get Velocities
    ---2d: Autonomous Targets
    ---2e: Robot Relative Deltas
    ---2f: Field Relative Deltas

    -------------------------------
    --------MUTATOR METHODS--------
    -------------------------------

    --1: Pose Mutator

    */


    //1: General Telemetry Accessors
    public double getAutoCounter() {
        return autoCounter;
    }

    public double getRotationRemaining() {
        return rotationRemaining;
    }


     /*
    --------------------------------
    -2: ODOMETRY SPECIFIC ACCESSORS-
    --------------------------------
    */

    //2a: Raw Counts to Inches
    //Also where we get the encoder counts from

    public double getXDistance() {
        double xDistance = (((frontLeft.getCurrentPosition()-backRight.getCurrentPosition())+(frontRight.getCurrentPosition()-backLeft.getCurrentPosition()))/4);
        xDistance = xDistance / getDriveCPI();
        return xDistance;
    }

    public double getYDistance() {
        double yDistance = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition()) / 4;
        yDistance = yDistance / getDriveCPI();
        return yDistance;
    }


    //2b: CURRENT POSITION

    //Returns Current X position
    public double getPoseX() {
        return poseX;
    }

    //Returns Current Y position
    public double getPoseY() {
        return poseY;
    }

    // 2c: WHEEL VELOCITIES

    private double lastFL = 0;
    private double lastFR = 0;
    private double lastBL = 0;
    private double lastBR = 0;

    public double getFLVelo() {
        double velocity = ((frontLeft.getCurrentPosition()-lastFL)/ getDriveCPI())/veloTimer.time();
        lastFL = frontLeft.getCurrentPosition();
        return velocity;
    }

    public double getFRVelo() {
        double velocity = ((frontRight.getCurrentPosition()-lastFR)/ getDriveCPI())/veloTimer.time();
        lastFR = frontRight.getCurrentPosition();
        return velocity;
    }

    public double getBLVelo() {
        double velocity = ((backLeft.getCurrentPosition()-lastBL)/ getDriveCPI())/veloTimer.time();
        lastBL = backLeft.getCurrentPosition();
        return velocity;
    }

    public double getBRVelo() {
        double velocity = ((backRight.getCurrentPosition()-lastFR)/ getDriveCPI())/veloTimer.time();
        lastBR = backRight.getCurrentPosition();
        return velocity;
    }

    // 2d: AUTONOMOUS TARGETS

    //Returns the X position targeted by the autonomous
    public double getTargetX() {
        return targetX;
    }

    //Returns the Y position targeted by the autonomous
    public double getTargetY() {
        return targetY;
    }


    //2e: ROBOT RELATIVE DELTAS

    //Gets the change in position in relation to the ROBOT on the X axis
    public double getRobotRelativeDeltaX() {
        return deltaRobotX;
    }

    //Gets the change in position in relation to the ROBOT on the Y axis
    public double getRobotRelativeDeltaY() {
        return deltaRobotY;
    }

    //FIELD RELATIVE DELTAS
    //Gets the change in position in relation to the FIELD on the X axis
    public double getFieldRelativeDeltaX() {
        return deltaFieldX;
    }

    //Gets the change in position in relation to the FIELD on the Y axis
    public double getFieldRelativeDeltaY() {
        return deltaFieldY;
    }


    //MUTATOR METHODS

    //1: Pose Mutator
    //Call this to manually set the pose of the robot
    public void setPose(double poseX, double poseY) {
        this.poseX = poseX;
        this.poseY = poseY;
    }

}