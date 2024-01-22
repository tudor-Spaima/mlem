package org.firstinspires.ftc.teamcode.mlem;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class commandbase extends Hardware {



    public commandbase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void setMotorPosition(int position, DcMotorEx motor) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

    }


    public void moveBrat(double position) {
        brat_l.setPosition(position);
        brat_r.setPosition(position );
    }


    public void movePivot(double position) {
        pivot.setPosition(position);
    }


    public void botSAFE() {
        moveBrat(CONSTANTS.BRAT_SAFE);
        movePivot(CONSTANTS.PIVOT_SAFE);
    }

    public void intakePos(){
        moveBrat(CONSTANTS.BRAT_INTAKE);
        movePivot(CONSTANTS.PIVOT_INTAKE);


    }

    public void scorePos(){
        moveBrat(CONSTANTS.BRAT_SCORE);
        movePivot(CONSTANTS.PIVOT_SCORE);
    }



    public void bratJos(boolean ok){
        if(ok){
            moveBrat(CONSTANTS.BRAT_INTAKE);

        }else {
            moveBrat(CONSTANTS.BRAT_SAFE);

        }
    }

    public  boolean leftSensor(){
        if(dist_l.getDistance(DistanceUnit.CM)<= 4.5)
            return true;
        else return false;

    }

    public boolean rightSensor(){
        if(dist_r.getDistance(DistanceUnit.CM)<= 6)
            return true;
        else return false;

    }

    public void swing( CONSTANTS.swing_direction direction){
        if(direction == CONSTANTS.swing_direction.up){
            movePivot(CONSTANTS.PIVOT_SCORE);
            claw_l.setPosition(CONSTANTS.CLAW_L_CLOSED);
            claw_r.setPosition(CONSTANTS.CLAW_R_CLOSED);
            moveBrat(CONSTANTS.BRAT_SCORE);

        }else{
            movePivot(CONSTANTS.PIVOT_INTAKE);


            moveBrat(CONSTANTS.BRAT_INTAKE);


        }
    }



    public  void controlSlider(int pos){
        setMotorPosition(pos, slider_r);
       setMotorPosition(pos, slider_l);

    }


}
