package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class TeamDrivetrain {


    //Defines the motor variables (ex: frontLeft) as a DcMotor
    public Motor frontLeft;
    public Motor backLeft;
    public Motor frontRight;
    public Motor backRight;

    public Motor.Encoder flEncoder;
    public Motor.Encoder blEncoder;
    public Motor.Encoder frEncoder;
    public Motor.Encoder brEncoder;

    public MecanumDrive mecanum;

    public ChassisSpeeds speeds;

    public double frontLeftPower;
    public double backLeftPower;
    public double frontRightPower ;
    public double backRightPower;


    private RevIMU imu;



    private double radHeading;
    private double degHeading;


    //IMPORTANT FOR ENCODER CALCULATIONS
    //CPR = Counts Per Revolution
    //IPC = Inches Per Count
    private static final double gearRatio =22;
    private static final double motorCPR = 28;
    private static final double wheelDiameter = 3;

    //Adjusted Values in calculations
    private static final double adjustedCPR = motorCPR*gearRatio;
    private static double driveIPC = (Math.PI * wheelDiameter)/adjustedCPR;


    /*
    Call this at the start of your OpMode and it will Initialize the following:
    Drive Motors
    IMU/Gyro
     */
    public void initDrivetrain(HardwareMap hwMap) {

        //Drive Motor Initialization

        //Maps the Motors to the proper slot on the RevHubs. Name is found in the configuration menu
        frontLeft = hwMap.get(Motor.class, "frontLeft");
        backLeft = hwMap.get(Motor.class, "backLeft");
        frontRight = hwMap.get(Motor.class, "frontRight");
        backRight = hwMap.get(Motor.class, "backRight");

        //Sets the right side to reverse
        frontRight.setInverted(true);
        backRight.setInverted(true);

        //Sets the motors to Brake when stopped
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        //Prevents movement of motors, just in case.
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();


        //ENCODER INITIALIZATION
        flEncoder = frontLeft.encoder;
        blEncoder = backLeft.encoder;
        frEncoder = frontRight.encoder;
        brEncoder = backRight.encoder;

        //Sets Inches Per Count (Pulse) of the Motors
        flEncoder.setDistancePerPulse(driveIPC);
        blEncoder.setDistancePerPulse(driveIPC);
        frEncoder.setDistancePerPulse(driveIPC);
        brEncoder.setDistancePerPulse(driveIPC);

        //IMU (Gyroscope) initialization

        //Maps the IMU/Gyroscope to the correct spot in the configuration

        imu = new RevIMU(hwMap,"imu");
        imu.init();

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        //Kinematics and Odometry
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                2.0,2.0,Math.PI / 2.0, Rotation2d.fromDegrees(45)
        );
    }

    public void stopDrive(){
        mecanum.stop();
    }

    //Mecanum Driving
    public void robotDrive(double x, double y, double rot, double speedAdj) {
        x *= speedAdj;
        y *= speedAdj;
        rot *= speedAdj;
        mecanum.driveRobotCentric(x,y,rot);
    }

    //Field Centric Driving
    public void fieldDrive(double x, double y, double rot, double speedAdj){

        //Gets the angle of the robot in radians
        //Used to determine relative rotation in relation to field
        double heading = getHeadingInDegrees();

        x *= speedAdj;
        y *= speedAdj;
        rot *= speedAdj;

        //Calls the Robot drive function, now using the updated inputs for field oriented
        mecanum.driveFieldCentric(x,y,rot,heading);


    }

    //Returns gyro angle in respective unit

    public double getHeadingInRadians() {
        //radHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return radHeading;

    }

    public double getHeadingInDegrees() {
        //degHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return degHeading;
    }

}
