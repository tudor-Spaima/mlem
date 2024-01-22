package org.firstinspires.ftc.teamcode.mlem;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mlem.CONSTANTS;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    DcMotorEx slider_r,slider_l;
    Servo  pivot, brat_l, brat_r, claw_l, claw_r, aveon;

    DistanceSensor distanta;
    RevColorSensorV3  dist_l,dist_r, offset;


    public Hardware(HardwareMap hardwareMap) {

        slider_r = hardwareMap.get(DcMotorEx.class, "slider_r");
        slider_l = hardwareMap.get(DcMotorEx.class, "slider_l");
        aveon = hardwareMap.get(Servo.class, "aveon");



        pivot = hardwareMap.get(Servo.class, "pivot");
        brat_l = hardwareMap.get(Servo.class, "brat_l");
        brat_r = hardwareMap.get(Servo.class, "brat_r");

        claw_r = hardwareMap.get(Servo.class, "claw_r");
        claw_l = hardwareMap.get(Servo.class, "claw_l");


        distanta = hardwareMap.get(DistanceSensor.class, "a");
        dist_l = hardwareMap.get(RevColorSensorV3.class, "dist_l");
        dist_r = hardwareMap.get(RevColorSensorV3.class, "dist_r");
        offset = hardwareMap.get(RevColorSensorV3.class, "offset");




        slider_l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider_l.setDirection(DcMotorSimple.Direction.REVERSE);


        slider_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider_r.setDirection(DcMotorSimple.Direction.FORWARD);





    }







}
