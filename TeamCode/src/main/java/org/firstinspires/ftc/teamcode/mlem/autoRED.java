package org.firstinspires.ftc.teamcode.mlem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "autoRED")
public class autoRED extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        commandbase cb = new commandbase(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());



        cb.controlIntake(CONSTANTS.intake_states.retracted);
        cb.controlClaw(CONSTANTS.claw_states.closed);
        cb.safePos();




        drive.setPoseEstimate(new Pose2d(0,0,0));
        TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d( -3.7709931745785643,28.378883156705673, Math.toRadians(359.9650017023084
                )) )

                .build();
        TrajectorySequence colectar = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d( -16,  51 , Math.toRadians(180
                )) )

                .build();
        TrajectorySequence inter1 = drive.trajectorySequenceBuilder(colectar.end())
                .lineToLinearHeading(new Pose2d( 33.98596961335462, 52.50774232472559, Math.toRadians(180
                )) )


                .build();

        TrajectorySequence score3 = drive.trajectorySequenceBuilder(inter1.end())
                .lineToLinearHeading(new Pose2d( 66, 22, Math.toRadians(180
                )) )
                .addDisplacementMarker(()->{
                    cb.controlSlider(-CONSTANTS.SLIDE_SUS/2);
                    sleep(200);

                    cb.controlCupa(CONSTANTS.cupa_states.basculat);

                    cb.setMotorPosition(0, cb.intake_right);
                    cb.setMotorPosition(0, cb.intake_left);

                })

                .build();

        TrajectorySequence inter2 = drive.trajectorySequenceBuilder(score3.end())
                .lineToLinearHeading(new Pose2d( 33.98596961335462, 52.50774232472559, Math.toRadians(180
                )) )


                .build();

        TrajectorySequence colectar2 = drive.trajectorySequenceBuilder(inter2.end())
                .lineToLinearHeading(new Pose2d( -18.5,  49, Math.toRadians(180
                )) )
                .addTemporalMarker(1, ()->{
                    cb.moveBrat(CONSTANTS.BRAT_SAFE);


                })

                .build();

        TrajectorySequence inter3 = drive.trajectorySequenceBuilder(colectar2.end())
                .lineToLinearHeading(new Pose2d( 33.98596961335462, 52.50774232472559, Math.toRadians(180
                )) )

                .addTemporalMarker(0, ()->{
                    try {
                        cb.transfer();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                })


                .build();

        TrajectorySequence sccaorea = drive.trajectorySequenceBuilder(inter3.end())
                .lineToLinearHeading(new Pose2d( 64, 31, Math.toRadians(180
                )) )
                .addDisplacementMarker(()->{
                    cb.controlCioc(CONSTANTS.ciock_states.closed);
                    cb.controlSlider(-CONSTANTS.SLIDE_SUS);
                    sleep(200);
                    cb.controlCupa(CONSTANTS.cupa_states.basculat);



                })

                .build();

        TrajectorySequence parking = drive.trajectorySequenceBuilder(sccaorea.end())
                .lineToLinearHeading(new Pose2d( 64, 49.93744761589659, Math.toRadians(180
                )))
                .build();

        cb.controlCioc(CONSTANTS.ciock_states.closed);






        runtime.reset();
        waitForStart();
        runtime.startTime();


        cb.movePivot(CONSTANTS.PIVOT_INTAKE);


        drive.followTrajectorySequence(preload);
        cb.moveBrat(CONSTANTS.BRAT_INTAKE  );
        sleep(500);

        cb.controlClaw(CONSTANTS.claw_states.open);
        sleep(200);
        cb.safePos();





        drive.followTrajectorySequence(colectar);
        drive.followTrajectorySequence(inter1);
        drive.followTrajectorySequence(score3);


        cb.controlCioc(CONSTANTS.ciock_states.open);
        sleep(400);
        cb.controlCupa(CONSTANTS.cupa_states.transfer);
        sleep(200);
        cb.controlSlider(0);

        drive.followTrajectorySequence(inter2);


        drive.followTrajectorySequence(colectar2);
        cb.movePivot(CONSTANTS.PIVOT_STACK);
        cb.setMotorPosition(-CONSTANTS.INTAKE_EXTEND, cb.intake_right);
        cb.setMotorPosition(-CONSTANTS.INTAKE_EXTEND, cb.intake_left);
        cb.moveBrat(0.9);
        sleep(500);


        cb.controlClaw(CONSTANTS.claw_states.closed);
        sleep(500);


        drive.followTrajectorySequence(inter3);
        drive.followTrajectorySequence(sccaorea);
        sleep(200);

        cb.controlCioc(CONSTANTS.ciock_states.open);
        sleep(400);
        cb.controlCupa(CONSTANTS.cupa_states.transfer);



        cb.controlSlider(0);
        drive.followTrajectorySequence(parking);








        telemetry.addData("timp ramas", (runtime.seconds()));
        telemetry.update();

        sleep(2000);










    }
}
