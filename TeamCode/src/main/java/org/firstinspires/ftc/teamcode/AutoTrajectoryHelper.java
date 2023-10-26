package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;

public class AutoTrajectoryHelper {



    Pose2d startPosition;
    Pose2d endPosition;

    Translation2d interiorPoints;


    TrajectoryConfig configuration = new TrajectoryConfig(
            inchesToMeters(TeamDrivetrain.maxVelocity),
            inchesToMeters(TeamDrivetrain.maxAccel)
    );


    public static Pose2d generatePose2d(double x, double y,double rot) {
        return new Pose2d(inchesToMeters(x), inchesToMeters(y), Rotation2d.fromDegrees(rot));
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public void setStart(double x, double y, double rotation) {
        startPosition =  generatePose2d(x,y,rotation);
    }

    public void setEnd(double x, double y, double rotation){
        endPosition =  generatePose2d(x,y,rotation);
    }
}
