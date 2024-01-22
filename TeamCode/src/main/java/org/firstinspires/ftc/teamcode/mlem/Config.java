package org.firstinspires.ftc.teamcode.mlem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.acmerobotics.dashboard.config.Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "config")
public class Config extends LinearOpMode {
    public static  double CLAW_L = 0;
    public static  double CLAW_R = 1;
    public static  double BRAT_L = 0.95;
    public static  double BRAT_R = 1;
    public static  double PIVOT = 0.12;

    public static  int SLIDE = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        Hardware hardware = new Hardware(hardwareMap);
        commandbase cb = new commandbase(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()){



                hardware.brat_l.setPosition(BRAT_L);
                hardware.brat_r.setPosition(BRAT_L);

                hardware.claw_l.setPosition(CLAW_L);
                hardware.claw_r.setPosition(CLAW_R);

                //cb.controlSlider(SLIDE);





            hardware.pivot.setPosition(PIVOT);

            telemetry.addData("touchpad pressed", gamepad2.touchpad);



            telemetry.addData("slider_l", hardware.slider_l.getCurrentPosition());
            telemetry.addData("slider_r", hardware.slider_r.getCurrentPosition());

            telemetry.addData("distanta", cb.distanta.getDistance(DistanceUnit.CM));
            telemetry.addData("dist_l", cb.dist_l.getDistance(DistanceUnit.CM));
            telemetry.addData("dist_r", cb.dist_r.getDistance(DistanceUnit.CM));


                telemetry.update();




        }
        
    }
}
