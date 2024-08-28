package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.List;

public class SlidesBoard {


    public DcMotorEx slidesMotor;

    public void init(HardwareMap hardwareMap) {

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPower(1);
    }
    public void setPosition(int position){
        slidesMotor.setTargetPosition(position);
        slidesMotor.setPower(1);
    }
    public final class SlidesToPosition implements Action {
        int Position;
        boolean cancel=false;
        Telemetry tel;
        public void CancelSlides(){
            cancel=true;

        }
    public SlidesToPosition (int Position, Telemetry tel){
        this.Position = Position;
        setPosition(Position);
        this.tel=tel;
    }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (Math.abs(slidesMotor.getCurrentPosition()-Position) < 25) {
                return false;
            }

            if (cancel==true) {
                setPosition(slidesMotor.getCurrentPosition());
                return false;
            }


            setPosition(Position);

            tel.addData("Target Position", slidesMotor.getTargetPosition());
            tel.addData("Current Power", slidesMotor.getPower());
            tel.addData("Target Position 2", Position);
            tel.addData("Motor current position", slidesMotor.getCurrentPosition());

            tel.update();

            return true;
        }
    }

    //cancellableSlides implents cancle actions for the slides class
    public final class CancellableSlides implements Action {
        final Gamepad gamepad;
        final Action action;
        public CancellableSlides (Action action,Gamepad gamepad) {
            this.action = action;
            this.gamepad = gamepad;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(gamepad.a==true) {
                setPosition(slidesMotor.getCurrentPosition());
                return false;
            }
            return action.run(telemetryPacket);

        }
    }
    public final class TouchCancleSlides implements Action{
        final Action action;
        final TouchSensor touchSensor;


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(touchSensor.isPressed()){
                setPosition(slidesMotor.getCurrentPosition());
                return false;
            }
            return action.run(telemetryPacket);
        }
        public TouchCancleSlides (Action action, TouchSensor touchSensor){
            this.action = action;
            this.touchSensor = touchSensor;
        }
    }




}

