package org.firstinspires.ftc.teamcode.mlem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.acmerobotics.dashboard.config.Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "config")
public class Config extends LinearOpMode {
    public static  double CLAW = 0;
    public static  double CUPA = 0;
    public static  double BRAT_L = 0;
    public static  double BRAT_R = BRAT_L+0.02;
    public static  double PIVOT = 1;
    public static  double CIOC = 1;



    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);
        commandbase cb = new commandbase(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()){

                hardware.claw.setPosition(CLAW);

                hardware.cupa.setPosition(CUPA);

                hardware.brat_l.setPosition(BRAT_L);
                hardware.brat_r.setPosition( BRAT_L+0.03);

                hardware.cioc.setPosition(CIOC);


            hardware.pivot.setPosition(PIVOT);
//            if(gamepad1.left_bumper) {            cb.controlSlider(CONSTANTS.SLIDE_SUS);}
//            if(gamepad1.right_bumper) {            cb.controlSlider(0);}
//


            telemetry.addData("slider_l", hardware.slider_l.getCurrentPosition());
            telemetry.addData("slider_r", hardware.slider_r.getCurrentPosition());
            telemetry.addData("ex_l", hardware.intake_left.getCurrentPosition());
            telemetry.addData("ex_r", hardware.intake_right.getCurrentPosition());
            telemetry.addData("distanta", cb.distanta.getDistance(DistanceUnit.CM));


                telemetry.update();




        }
        
    }
}
