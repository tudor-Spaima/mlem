package org.firstinspires.ftc.teamcode.mlem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistancePIDController {

    private static final double Kp = 1;
    private static final double Ki = 0;
    private static final double Kd = 1;

    private static final double DESIRED_DISTANCE = 10;
    private static final double MOTOR_POWER_LIMIT = 1;
    private static final double SENSOR_OFFSET = 0;

    private double integral = 0;
    private double previousError = 0;

    private DistanceSensor distanceSensor;
    private DcMotorEx motor;

    private ElapsedTime runtime = new ElapsedTime();

    public DistancePIDController(DistanceSensor sensor, DcMotorEx motor) {
        this.distanceSensor = sensor;
        this.motor = motor;
    }

    private void pidControlLoop() {
        double currentDistance = this.distanceSensor.getDistance(DistanceUnit.CM) - SENSOR_OFFSET;
        double error = DESIRED_DISTANCE - currentDistance;
        integral += error;
        double derivative = error - previousError;

        double output = Kp * error + Ki * integral + Kd * derivative;

        double motorPower = Math.min(Math.max(-MOTOR_POWER_LIMIT, output), MOTOR_POWER_LIMIT);

        this.motor.setPower(motorPower);

        previousError = error;
    }

    public void controlLoop() {
        pidControlLoop();
    }
}
