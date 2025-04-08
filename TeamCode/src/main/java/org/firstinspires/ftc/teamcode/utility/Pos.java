package org.firstinspires.ftc.teamcode.utility;

public class Pos {
    public double x, y, r;

    public Pos(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public Pos (double x, double y) {
        this.x = x;
        this.y = y;
        this.r = 0;
    }

    public Pos() {
        this.x = 0;
        this.y = 0;
        this.r = 0;
    }

    //Accessors
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getR() { return r; }

    //Mutators
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setR(double r) { this.r = r; }
    public void set(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public static Pos add(Pos first, Pos second) {
        return new Pos(
                first.x + second.x,
                first.y + second.y,
                first.r + second.r
        );
    }

    public static Pos subtract(Pos first, Pos second) {
        return new Pos(
                first.x - second.x,
                first.y - second.y,
                first.r - second.r
        );
    }

    public static Pos multiply(Pos first, Pos second) {
        return new Pos(
                first.x * second.x,
                first.y * second.y,
                first.r * second.r
        );
    }
}