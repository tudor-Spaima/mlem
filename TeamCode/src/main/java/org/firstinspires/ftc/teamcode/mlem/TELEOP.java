package org.firstinspires.ftc.teamcode.mlem;

;

import static com.outoftheboxrobotics.photoncore.PhotonCore.photon;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


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

@Photon
@Config
@TeleOp(name = "TELEOP")
public class TELEOP extends LinearOpMode {

    public enum states {SAFE, INTAKE, SCORE};

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TELEOP.states state = TELEOP.states.SAFE;
        CONSTANTS.claw_states claw_state = CONSTANTS.claw_states.open;


        ElapsedTime runtime = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Thread systemControllerThread = new Thread(new SystemsController(telemetry,  gamepad1, drive));
        commandbase cb = new commandbase(hardwareMap);


//        PhotonCore photonCore = new PhotonCore();
//        photonCore.onOpModePreStart(this);

        //photonCore.onOpModePreStart(this);


        cb.safePos();
        waitForStart();


        systemControllerThread.start();
        while (opModeIsActive()){

            telemetry.addData("runtime: ", runtime.seconds());
            telemetry.addData("state ", state);
            telemetry.addData("slider", cb.slider_r.getCurrentPosition());

            telemetry.update();



            switch (state) {

                case SAFE:

                    if(gamepad2.dpad_left){
                        cb.intakePos();
                        state = TELEOP.states.INTAKE;
                    }

                    break;
                case INTAKE:
                    cb.controlCioc(CONSTANTS.ciock_states.open);


                    if (gamepad2.left_bumper) {
                        cb.controlIntake(CONSTANTS.intake_states.extended);
                    }
                    if (gamepad2.right_bumper) {
                        cb.controlIntake(CONSTANTS.intake_states.retracted);
                    }

                    if(gamepad1.right_bumper){
                        if(claw_state == CONSTANTS.claw_states.closed ){
                            claw_state = CONSTANTS.claw_states.open;
                        }else{
                            claw_state = CONSTANTS.claw_states.closed;

                                                    }
                        sleep(200);
                    }

                    cb.controlClaw(claw_state);
                    if(gamepad2.left_trigger != 0){
                        cb.moveBrat(CONSTANTS.BRAT_SAFE);
                        cb.movePivot(CONSTANTS.PIVOT_SAFE);

                    }else{
                        cb.bratJos(gamepad1.left_trigger!=0);
                        cb.movePivot(CONSTANTS.PIVOT_INTAKE);


                    }



                    if(gamepad2.dpad_right){
                        cb.transfer();
                        sleep(200);
                        state = TELEOP.states.SCORE;


                    }


                    break;
                case SCORE:

                    if(gamepad2.dpad_up ) {
                        cb.controlCioc(CONSTANTS.ciock_states.closed);
                        cb.controlSlider(CONSTANTS.SLIDE_SUS);
                        sleep(200);
                        cb.controlCupa(CONSTANTS.cupa_states.basculat);

                    }

                    if(gamepad1.right_trigger!=0){
                        cb.controlCioc(CONSTANTS.ciock_states.open);

                    }else {
                        cb.controlCioc(CONSTANTS.ciock_states.closed);

                    }

                    if(gamepad2.right_trigger!=0)    {

                        cb.controlCupa(CONSTANTS.cupa_states.transfer);
                        cb.intakePos();
                        claw_state = CONSTANTS.claw_states.open;

                        cb.controlSlider(0);

                        state = TELEOP.states.INTAKE;

                    }

                    break;
            }

        }


    }

}
