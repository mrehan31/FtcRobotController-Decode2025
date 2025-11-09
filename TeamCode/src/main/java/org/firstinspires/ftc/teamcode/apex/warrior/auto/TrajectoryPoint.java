package org.firstinspires.ftc.teamcode.apex.warrior.auto;

public class TrajectoryPoint {
    public Pose2d pose;
    public double time;

    public TrajectoryPoint(Pose2d pose, double time) {
        this.pose = pose;
        this.time = time;
    }
}
