package org.firstinspires.ftc.teamcode.learnJava;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class basicDriveControlsV1 extends OpMode{
    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if( gamepad1.left_stick_y != 0){
            leftFront.setPower(0.6*gamepad1.left_stick_y);
            leftBack.setPower(0.6*gamepad1.left_stick_y);
            rightBack.setPower(0.6*gamepad1.left_stick_y);
            rightFront.setPower(0.6*gamepad1.left_stick_y);

        }else if( gamepad1.left_stick_x != 0){
            leftFront.setPower(-0.6*gamepad1.left_stick_x);
            leftBack.setPower(0.6*gamepad1.left_stick_x);
            rightBack.setPower(-0.6*gamepad1.left_stick_x);
            rightFront.setPower(0.6*gamepad1.left_stick_x);

        }else if( gamepad1.right_stick_x != 0){
            leftFront.setPower(-0.6*gamepad1.right_stick_x);
            leftBack.setPower(-0.6*gamepad1.right_stick_x);
            rightBack.setPower(0.6*gamepad1.right_stick_x);
            rightFront.setPower(0.6*gamepad1.right_stick_x);

        }else{
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

        }

    }

}
