package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class drive
{
    int startX, startY;
    int currentX, currentY;

    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    Telemetry tel;

    public drive(int startX, int startY, HardwareMap hardwareMap, Telemetry tel)
    {
        this.startX=startX;
        this.startY=startY;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.tel=tel;
    }

    public void foward(){

        leftFront.setTargetPosition(500);
        leftBack.setTargetPosition(500);
        rightFront.setTargetPosition(500);
        rightBack.setTargetPosition(500);
        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);


        while(Math.abs(leftFront.getCurrentPosition() -500) <30)
        {
            tel.addData("", leftFront.getPower());
            tel.addData("", leftFront.getCurrentPosition());
            tel.update();
        }

    }

    public int getX()
    {
        return currentX;
    }
}
