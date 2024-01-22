package org.firstinspires.ftc.teamcode.mlem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



class SystemsController implements Runnable {
    // Your systems control methods
    Telemetry telemetry;
    SampleMecanumDrive drive ;
    Gamepad gp1;

    public SystemsController(Telemetry telemetry, Gamepad gp1, SampleMecanumDrive drive) {
        this.telemetry = telemetry;
        this.gp1 = gp1;
        this.drive = drive;
    }
    @Override
    public void run() {
        while (true) {
            this.drive.setWeightedDrivePower(
                    new Pose2d(
                            this.gp1.left_stick_y,
                            this.gp1.left_stick_x,
                            this.gp1.right_stick_x
                    )
            );

        }
    }
}

@TeleOp(name = "TELEOP")
public class TELEOP extends LinearOpMode {

    private enum states {SAFE, INTAKE, SCORE};
    private enum modes {NORMAL, FAIL};

    private  enum drive_modes {};



    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TELEOP.states state = TELEOP.states.SAFE;
        TELEOP.modes mode = TELEOP.modes.NORMAL;

        CONSTANTS.claw_states claw_l_state = CONSTANTS.claw_states.open;
        CONSTANTS.claw_states claw_r_state = CONSTANTS.claw_states.open;







        ElapsedTime runtime = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Thread systemControllerThread = new Thread(new SystemsController(telemetry,  gamepad1, drive));
        commandbase cb = new commandbase(hardwareMap);


        PhotonCore photonCore = new PhotonCore();
        PhotonCore.ExperimentalParameters ph = new PhotonCore.ExperimentalParameters();
        ph.setMaximumParallelCommands(8);
        ph.setSinglethreadedOptimized(false);

        cb.botSAFE();



        waitForStart();


        systemControllerThread.start();
        while (opModeIsActive()) {

            telemetry.addData("STATE ", state);
            telemetry.addData("MODE ", mode);

            telemetry.addData("DISTANCE (CM) ", cb.distanta.getDistance(DistanceUnit.CM));
            telemetry.addData("DISTANCE_L (CM) ", cb.dist_l.getDistance(DistanceUnit.CM));
            telemetry.addData("DISTANCE_R (CM) ", cb.dist_r.getDistance(DistanceUnit.CM));

            telemetry.addData("slider_r", cb.slider_r.getCurrentPosition());
            telemetry.addData("slider_l", cb.slider_l.getCurrentPosition());

            telemetry.addData("touchpad1 pressed", gamepad1.touchpad);
            telemetry.addData("touchpad2 pressed", gamepad2.touchpad);



            telemetry.update();

            switch (mode) {
                case NORMAL:


                    switch (state) {

                        case SAFE:

                            if (gamepad2.dpad_left) {
                                cb.intakePos();
                                state = TELEOP.states.INTAKE;
                                //open claws
                                cb.claw_l.setPosition(CONSTANTS.CLAW_L_OPEN);
                                cb.claw_r.setPosition(CONSTANTS.CLAW_R_OPEN);
                                claw_l_state = CONSTANTS.claw_states.open;
                                claw_r_state = CONSTANTS.claw_states.open;

                            }

                            break;

                        case INTAKE:
                            if (gamepad2.left_trigger != 0) {
                                cb.movePivot(CONSTANTS.PIVOT_SAFE);

                            } else {
                                cb.movePivot(CONSTANTS.PIVOT_INTAKE);
                                cb.bratJos(gamepad1.left_trigger != 0);
                            }



                            if (gamepad1.left_bumper) {
                                if (claw_l_state == CONSTANTS.claw_states.open) {
                                    cb.claw_l.setPosition(CONSTANTS.CLAW_L_CLOSED);
                                    claw_l_state = CONSTANTS.claw_states.closed;
                                    sleep(200);
                                } else {
                                    cb.claw_l.setPosition(CONSTANTS.CLAW_L_OPEN);
                                    claw_l_state = CONSTANTS.claw_states.open;
                                    sleep(200);
                                }
                            }

                            if (gamepad1.right_bumper) {
                                if (claw_r_state == CONSTANTS.claw_states.open) {
                                    cb.claw_r.setPosition(CONSTANTS.CLAW_R_CLOSED);
                                    claw_r_state = CONSTANTS.claw_states.closed;
                                    sleep(200);
                                } else {
                                    cb.claw_r.setPosition(CONSTANTS.CLAW_R_OPEN);
                                    claw_r_state = CONSTANTS.claw_states.open;
                                    sleep(200);
                                }
                            }
                            if(cb.leftSensor()){
                                cb.claw_l.setPosition(CONSTANTS.CLAW_L_CLOSED);
                                claw_l_state = CONSTANTS.claw_states.closed;

                            }
                            if (cb.rightSensor()) {
                                cb.claw_r.setPosition(CONSTANTS.CLAW_R_CLOSED);
                                claw_r_state = CONSTANTS.claw_states.closed;
                            }

                            if(claw_l_state == CONSTANTS.claw_states.closed && claw_l_state == CONSTANTS.claw_states.closed ){
                                sleep(200);
                                cb.botSAFE();
                                state = states.SCORE;

                            }

                            if(gamepad2.circle){
                                cb.botSAFE();
                                state = states.SCORE;
                            }
                            break;

                            case  SCORE:

                                if(gamepad2.triangle){
                                    cb.controlSlider(CONSTANTS.SLIDE_UP);
                                    sleep(3000);
                                    cb.swing(CONSTANTS.swing_direction.up);
                                }

                                if (gamepad1.left_bumper) {
                                        cb.claw_l.setPosition(CONSTANTS.CLAW_L_OPEN);
                                        claw_l_state = CONSTANTS.claw_states.open;
                                }
                                if(gamepad1.right_bumper){
                                    cb.claw_r.setPosition(CONSTANTS.CLAW_R_OPEN);
                                    claw_r_state = CONSTANTS.claw_states.open;
                                }
                                if(gamepad2.cross){
                                    cb.claw_r.setPosition(CONSTANTS.CLAW_R_CLOSED);
                                    cb.claw_l.setPosition(CONSTANTS.CLAW_L_CLOSED);
                                    claw_l_state = CONSTANTS.claw_states.closed;
                                    claw_r_state = CONSTANTS.claw_states.closed;

                                    sleep(300);
                                    cb.swing(CONSTANTS.swing_direction.down);
                                    sleep(1000);
                                    cb.controlSlider(0);
                                    state = states.INTAKE;
                                }


                    }




                    if(gamepad2.touchpad){
                        mode = TELEOP.modes.FAIL;
                    }


                    break;


                case FAIL:

                    cb.slider_l.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                    cb.slider_r.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

                    if (gamepad1.left_bumper) {
                        if (claw_l_state == CONSTANTS.claw_states.open) {
                            cb.claw_l.setPosition(CONSTANTS.CLAW_L_CLOSED);
                            claw_l_state = CONSTANTS.claw_states.closed;
                        } else {
                            cb.claw_l.setPosition(CONSTANTS.CLAW_L_OPEN);
                            claw_l_state = CONSTANTS.claw_states.open;
                        }
                    }

                    if (gamepad1.right_bumper) {
                        if (claw_r_state == CONSTANTS.claw_states.open) {
                            cb.claw_r.setPosition(CONSTANTS.CLAW_R_CLOSED);
                            claw_r_state = CONSTANTS.claw_states.closed;
                        } else {
                            cb.claw_r.setPosition(CONSTANTS.CLAW_R_OPEN);
                            claw_r_state = CONSTANTS.claw_states.open;
                        }
                    }

                    if(gamepad2.touchpad){
                        cb.slider_l.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        cb.slider_r.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                        mode = TELEOP.modes.NORMAL;



                    }



                    break;




            }
        }

    }


}



