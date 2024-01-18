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

    public void controlIntake(CONSTANTS.intake_states state) {
        switch (state) {
            case extended:
                setMotorPosition(CONSTANTS.INTAKE_EXTEND, intake_right);
                setMotorPosition(CONSTANTS.INTAKE_EXTEND, intake_left);
                break;
            case retracted:
                setMotorPosition(0, intake_right);
                setMotorPosition(0, intake_left);
                break;

        }

    }


    public void moveBrat(double position) {
        brat_l.setPosition(position);
        brat_r.setPosition(position + 0.03);
    }


    public void movePivot(double position) {
        pivot.setPosition(position);
    }

    public void transferREADY() {
        controlIntake(CONSTANTS.intake_states.retracted);
        cupa.setPosition(CONSTANTS.CUPA_TRANSFER);
        moveBrat(CONSTANTS.BRAT_TRANSFER);
        movePivot(CONSTANTS.PIVOT_TRANSFER);
    }

    public void botSAFE() {
        moveBrat(CONSTANTS.BRAT_SAFE);
        movePivot(CONSTANTS.PIVOT_SAFE);
    }

    public void transfer() throws InterruptedException {
        //transferREADY();
        controlIntake(CONSTANTS.intake_states.retracted);


        cupa.setPosition(CONSTANTS.CUPA_TRANSFER);
        movePivot(CONSTANTS.PIVOT_TRANSFER);
        sleep(300);

        moveBrat(CONSTANTS.BRAT_TRANSFER);

        sleep(400);
        this.controlClaw(CONSTANTS.claw_states.transfer);
        sleep(400);
        botSAFE();

    }

    public void intakePos(){
        cupa.setPosition(CONSTANTS.CUPA_TRANSFER);
        moveBrat(CONSTANTS.BRAT_INTAKE);
        movePivot(CONSTANTS.PIVOT_INTAKE);

    }


    public void controlClaw(CONSTANTS.claw_states claw_state) {


            switch (claw_state) {
                case open:
                    claw.setPosition(CONSTANTS.CLAW_OPEN);
                    break;
                case closed:
                    claw.setPosition(CONSTANTS.CLAW_CLOSED);
                    break;

                    case transfer:
                    claw.setPosition(0.2);
                    break;


            }

    }

    public void controlCupa(CONSTANTS.cupa_states cupa_state){
        switch (cupa_state) {
            case transfer:
                cupa.setPosition(CONSTANTS.CUPA_TRANSFER);
                break;
            case basculat:
                cupa.setPosition(CONSTANTS.CUPA_BASCULAT);
                break;
        }
    }

    public void controlCioc(CONSTANTS.ciock_states cioc_state){
        switch (cioc_state) {
            case open:
                cioc.setPosition(CONSTANTS.CIOC_OPEN);
                break;
            case closed:
                cioc.setPosition(CONSTANTS.CIOC_CLOSED);
                break;
        }
    }
    public void bratJos(boolean ok){
        if(ok){
            moveBrat(CONSTANTS.BRAT_INTAKE);

        }else {
            moveBrat(CONSTANTS.BRAT_INTAKE - 0.04 );

        }
    }

    public void safePos(){
        moveBrat(CONSTANTS.BRAT_SAFE);
        movePivot(CONSTANTS.PIVOT_SAFE);
        cupa.setPosition(CONSTANTS.CUPA_TRANSFER);
    }

    public  void controlSlider(int pos){
        setMotorPosition(pos, slider_r);
        setMotorPosition(pos, slider_l);

    }

//    public  void autoSlider(double dist){
//        int error = CONSTANTS.INTAKE_EXTEND - intake_right.getCurrentPosition();
//
//
//        int d = (int) (CONSTANTS.INTAKE_EXTEND *(distanta.getDistance(DistanceUnit.CM) - dist));
//
//        setMotorPosition(-1*((int) ( intake_right.getCurrentPosition()+ d )),intake_right);
//        setMotorPosition((-1*(int) ( intake_left.getCurrentPosition()+d )),intake_left);
//    }
}
