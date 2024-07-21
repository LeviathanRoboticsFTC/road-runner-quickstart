package org.firstinspires.ftc.teamcode.learnJava;

public class Point {
    int x;
    int y;
    static int lifespan;

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }
    @Override
    public String toString(){
        return "Point " + x + " " + y;
    }
    public static int returnLifespan(){
        return lifespan;
    }
}
