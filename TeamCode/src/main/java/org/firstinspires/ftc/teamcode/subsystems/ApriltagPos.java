package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

public class ApriltagPos {
    private boolean found;
    private double a;
    private double b;
    private double c;
    public enum Status{
        ACTIVE,
        INACTIVE,
        NEITHER
    }
    private Status status;

    public ApriltagPos(double X, double Y, double Z) {
        found = false;
        a = X;
        b = Y;
        c = Z;
        status = Status.NEITHER;
    }

    public void setX(double n) {
        a = n;
    }

    public void setY(double n) {
        b = n;
    }

    public void setZ(double n) {
        c = n;
    }

    public void setStatus(Status status){
        this.status = status;
    }

    public boolean isFound(){
        return found;
    }

    public double x() {
        return a;
    }

    public double y() {
        return b;
    }

    public double z(){
        return c;
    }

    public Status status(){
        return status;
    }

//    public void update(double weight, double x, double y, double heading, double elevation) {
//        double cos = Math.cos(heading);
//        double sin = Math.sin(heading);
//        double dot = (a - x) * cos + (b - y) * sin;
//        double a2 = dot * cos + x;
//        double b2 = dot * sin + y;
//        a = (a * weight + a2) / (weight + 1);
//        b = (b * weight + b2) / (weight + 1);
//    }

    public void update(double alpha, double x, double y, double heading, double elevation, double dist){
        double phi = Math.PI/2 - elevation;
        double a2 = x + dist * Math.cos(heading) * Math.sin(phi);
        double b2 = y + dist * Math.sin(heading) * Math.sin(phi);
        double c2 = RobotConstants.CAMERA_HEIGHT + dist * Math.cos(phi);
        if(Double.isFinite(a2) && Double.isFinite(b2) && Double.isFinite(c2)) {
            found = true;
            a = a * (1 - alpha) + a2 * alpha;
            b = b * (1 - alpha) + b2 * alpha;
            c = c * (1 - alpha) + c2 * alpha;
        }
        if(found){
            if(c > 35) {
                status = Status.ACTIVE;
            }else if(c < 30){
                status = Status.INACTIVE;
            }else{
                status = Status.NEITHER;
            }
        }
    }

    public double findRange(double x, double y){
        return Math.sqrt(
                Math.pow(a - x, 2) + Math.pow(b - y, 2)
        );
    }
    public double findAngle(double x, double y) {
        return Math.toDegrees(Math.atan2((b - y), (a - x)));
    }

    @NonNull
    public String toString() {
        return "(" + Math.round(a * 100)/100.0 + "," + Math.round(b * 100)/100.0 + "," + Math.round(c * 100)/100.0 + ")";
    }
}