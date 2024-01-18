package org.firstinspires.ftc.teamcode.mlem;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mlem.CONSTANTS;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    DcMotorEx slider_r,slider_l, intake_left, intake_right;
    Servo  pivot, brat_l, brat_r, cupa, claw, cioc;

    DistanceSensor distanta ;

    public Hardware(HardwareMap hardwareMap) {

        slider_r = hardwareMap.get(DcMotorEx.class, "slider_r");
        slider_l = hardwareMap.get(DcMotorEx.class, "slider_l");

        intake_left = hardwareMap.get(DcMotorEx.class, "intake_left");
        intake_right = hardwareMap.get(DcMotorEx.class, "intake_right");

        pivot = hardwareMap.get(Servo.class, "pivot");
        brat_l = hardwareMap.get(Servo.class, "brat_l");
        brat_r = hardwareMap.get(Servo.class, "brat_r");

        cupa = hardwareMap.get(Servo.class, "cupa");
        claw = hardwareMap.get(Servo.class, "claw");
        cioc = hardwareMap.get(Servo.class, "cioc");

        distanta = hardwareMap.get(DistanceSensor.class, "a");







        intake_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_left.setDirection(DcMotorEx.Direction.FORWARD);


        intake_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_right.setDirection(DcMotorEx.Direction.REVERSE);


        slider_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider_l.setDirection(DcMotorSimple.Direction.REVERSE);

        slider_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider_r.setDirection(DcMotorSimple.Direction.FORWARD);





    }







}
