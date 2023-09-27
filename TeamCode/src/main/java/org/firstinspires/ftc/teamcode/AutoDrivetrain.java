package org.firstinspires.ftc.teamcode;

public class AutoDrivetrain extends TeamDrivetrain {

    //AUTONOMOUS DRIVE

    //Position/Odometry Variables
    private double poseX;
    private double poseY;

    private double deltaX;
    private double deltaY;

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
    private final double kSlipFactor = 0.5; //Deadzone adjustment for driving


    //CONSTRUCTORS
    public AutoDrivetrain() {
        poseX = 0;
        poseY = 0;
    }

    public AutoDrivetrain(double offsetX, double offsetY) {
        poseX = offsetX;
        poseY = offsetY;
    }

    //METHODS

    //Records Current Pose
    public void recordPose() {
        deltaX = getXDistance() - poseX;
        deltaY = getYDistance() - poseY;
        poseX += deltaX;
        poseY += deltaY;
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


        //Set Current, Initial, and Targeted Positions
        double currentRotation = getHeadingInDegrees(); //Records Rotation

        //On first run of this call:
        if(!autoDriveRunning) {
            initialXPosition = poseX; //Start X Pose when first called
            initialYPosition = poseY; //Start Y pose when first called
            initialRotation = currentRotation; //Start Rotation
            autoDriveRunning = true; //Makes sure robot knows this is running
            autoCounter ++;  //Counts the amount of 'First Calls' this has had
        }

        targetX = ((Math.sin(travelHeading) * distance) * kMecanumDrift) + initialXPosition; //Sets the targeted X position on the field
        targetY = ((Math.cos(travelHeading) * distance) * kMecanumDrift) + initialYPosition; //Sets the targeted Y position on the field

        double poseRatio; //Ratio between rotation difference and position difference
        if (distance != 0) poseRatio = Math.abs((desiredRotation-initialRotation)/(distance));  //Make sure its not dividing by 0
        else poseRatio = 1; //In case it divides by 0, its set to 1

        //INPUT VARIABLES
        double inputX; //Movement side-to-side
        double inputY; //Movement forward/back
        double inputRot; //Rotational Movement


        //DISTANCE CALCULATIONS
        //Sets the Input X and Ys after checking where the robot is in relation to it's targets. Deadzones set by the "Slip Factor"

        //Checks for if poseX is at targetX. if it isn't assign inputX after math
        if (poseX > targetX + kSlipFactor || poseX < targetX - kSlipFactor) inputX = Math.sin(travelHeading);
        else { //don't send inputX and flag it as atX
            inputX = 0;
            atX = true;
        }

        //Checks for if poseY is at targetY. if it isn't assign inputY after math
        if (poseY > targetY + kSlipFactor || poseY < targetY - kSlipFactor) inputY = Math.cos(travelHeading);
        else { //don't send inputY and flag it as atY
            inputY = 0;
            atY = true;
        }

        if (atY && atX) atDistance = true;//If we are at X and at Y, flag that we are at distance


        //ROTATION CALCULATIONS
        //Sets rotation speed according to poseRatio. Multiplies by kRotFactor for better control and adjustment
        if(currentRotation > desiredRotation + 3){
            inputRot = poseRatio * kRotFactor;
        }
        else if (currentRotation < desiredRotation - 3) {
            inputRot = -poseRatio * kRotFactor;
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


        //Telemetry Adjustments
        rotationRemaining = currentRotation-initialRotation;


        //If we are at the target position (both rot and distance), reset for next call
        if(atRotation && atDistance) atTarget = true;
        if(atTarget == true) autoDriveRunning = false;

        //Returns whether we are at target
        return atTarget;

    }

    //Returns a false if still running
    //Returns a true if it has stopped running
    //Give it a targetX and targetY on the field, and it drives to it
    //Speed range from 0 to 1
    //desiredRotation to set where the robot's heading will be once done
    public boolean autoDriveToPosition(double targetX, double targetY, double desiredSpeed, double desiredRotation) {
//        //Sets the boolean for if we are at targets
//        boolean atRotation = false; //Is rotation complete?
//
//        boolean atX = false; //Is it at targetX?
//        boolean atY = false; //Is it at targetY?
//        boolean atDistance = false; //Is it both at tX and tY?
//
//        boolean atTarget = false; //Robot at both targeted positions? (Stopped running)
//
//
//        //Set Current, Initial, and Targeted Positions
//        double currentRotation = getHeadingInDegrees(); //Records Rotation
//
//        //On first run of this call:
//        if(!autoDriveRunning) {
//            initialXPosition = poseX; //Start X Pose when first called
//            initialYPosition = poseY; //Start Y pose when first called
//            initialRotation = currentRotation; //Start Rotation
//            autoDriveRunning = true; //Makes sure robot knows this is running
//            autoCounter ++;  //Counts the amount of 'First Calls' this has had
//        }
//
//        double travelX = targetX - initialXPosition; //Distance Traveled on the X axis
//        double travelY = targetY - initialYPosition; //Distance Traveled on the Y axis
//        double travelHeading = Math.atan(travelX/travelY); //Theta (angle) of travel
//        double distance = travelX/Math.sin(travelHeading); //Total distance traveled
//
//        double poseRatio; //Ratio between rotation difference and position difference
//        if (distance != 0) poseRatio = Math.abs((desiredRotation-initialRotation)/(distance));  //Make sure its not dividing by 0
//        else poseRatio = 1; //In case it divides by 0, its set to 1
//
//        //INPUT VARIABLES
//        double inputX; //Movement side-to-side
//        double inputY; //Movement forward/back
//        double inputRot; //Rotational Movement
//
//
//        //DISTANCE CALCULATIONS
//        //Sets the Input X and Ys after checking where the robot is in relation to it's targets. Deadzones set by the "Slip Factor"
//
//        //Checks for if poseX is at targetX. if it isn't assign inputX after math
//        if (poseX > targetX + kSlipFactor || poseX < targetX - kSlipFactor) inputX = Math.sin(travelHeading);
//        else { //don't send inputX and flag it as atX
//            inputX = 0;
//            atX = true;
//        }
//
//        //Checks for if poseY is at targetY. if it isn't assign inputY after math
//        if (poseY > targetY + kSlipFactor || poseY < targetY - kSlipFactor) inputY = Math.cos(travelHeading);
//        else { //don't send inputY and flag it as atY
//            inputY = 0;
//            atY = true;
//        }
//
//        if (atY && atX) atDistance = true;//If we are at X and at Y, flag that we are at distance
//
//        //ROTATION CALCULATIONS
//        //Sets rotation speed according to poseRatio. Multiplies by kRotFactor for better control and adjustment
//        if(currentRotation > desiredRotation + 3){
//            inputRot = poseRatio * kRotFactor;
//        }
//        else if (currentRotation < desiredRotation - 3) {
//            inputRot = -poseRatio * kRotFactor;
//        }
//        else {
//            //ONLY RUNS WHEN WE ARE CORRECT ROTATION
//            atRotation = true;
//            inputRot = 0;
//        }
//
//        //DRIVE
//        //Call the Drive function using our inputs calculated
//        fieldDrive(inputY,inputX,inputRot,desiredSpeed);
//
//        //Telemetry Adjustments
//        rotationRemaining = currentRotation-initialRotation;
//
//
//        //If we are at the target position (both rot and distance), reset for next call
//        if(atRotation && atDistance) atTarget = true;
//        if(atTarget == true) autoDriveRunning = false;
//
//        //Returns whether we are at target
//        return atTarget;

        //On first run of this call:
        if(!autoDriveRunning) {
            initialXPosition = poseX; //Start X Pose when first called
            initialYPosition = poseY; //Start Y pose when first called
            autoDriveRunning = true; //Makes sure robot knows this is running
        }

        double travelX = targetX - initialXPosition; //Distance Traveled on the X axis
        double travelY = targetY - initialYPosition; //Distance Traveled on the Y axis
        double travelHeading = Math.atan(travelX/travelY); //Theta (angle) of travel
        double distance = travelX/Math.sin(travelHeading); //Total distance traveled

        return autoDriveToDistance(distance, desiredSpeed, desiredRotation, travelHeading, true);
    }

    public double getAutoCounter() {
        return autoCounter;
    }

    public double getRotationRemaining() {
        return rotationRemaining;
    }

    public double getXDistance() {
        double xDistance = (((frontLeft.getCurrentPosition()-backRight.getCurrentPosition())+(frontRight.getCurrentPosition()-backLeft.getCurrentPosition()))/4);
        xDistance /= getDriveCPI();
        return xDistance;
    }

    public double getYDistance() {
        double yDistance = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition())/4;
        yDistance /= getDriveCPI();
        return yDistance;
    }

    public double getPoseX(){
        return poseX;
    }

    public double getPoseY() {
        return poseY;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public void setPose(double poseX, double poseY) {
        this.poseX = poseX;
        this.poseY = poseY;
    }

}
