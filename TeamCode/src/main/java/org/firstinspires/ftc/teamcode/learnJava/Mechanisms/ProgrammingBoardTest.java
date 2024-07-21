package org.firstinspires.ftc.teamcode.learnJava.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProgrammingBoardTest {
    private DigitalChannel touchSensor;
    private DistanceSensor distanceSensor;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    public void init(HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public boolean getTouchSensorState() {
        return touchSensor.getState();
    }
    public double getDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }
    public void stopMotor(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }
    public void setDriveFowardSpeed(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
    }
}

