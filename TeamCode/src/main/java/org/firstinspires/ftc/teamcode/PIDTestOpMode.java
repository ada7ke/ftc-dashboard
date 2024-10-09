package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.EnumMap;

@SuppressWarnings("unused")
@Config
@TeleOp
public class PIDTestOpMode extends OpMode {

    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public EnumMap<MotorNames, DcMotor> motors;

    public double error;
    public double output;

    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public static double target = 50; // distance from sensor (50mm)
    private DistanceSensor sensorDistance;
    private MiniPID pid;

    @Override
    public void init() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        fl = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        fr = hardwareMap.get(DcMotor.class, "BACKLEFT");
        bl = hardwareMap.get(DcMotor.class, "FRONTRIGHT");
        br = hardwareMap.get(DcMotor.class, "FRONTLEFT");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid = new MiniPID(0.0, 0.0, 0.0);
        pid.setSetpoint(target);
        pid.setOutputLimits(-.5,.5);
        pid.setOutputRampRate(0.1);
    }

    @Override
    public void loop() {
        double distance = sensorDistance.getDistance(DistanceUnit.MM);
        output = pid.getOutput(distance, target);
        fl.setPower(output);
        fr.setPower(output);
        bl.setPower(output);
        br.setPower(output);
        double error = target - distance;
        telemetry.addData("output", output);
        telemetry.addData("distance", distance);
        telemetry.addData("target", target);
        telemetry.addData("error", error);
        telemetry.addData("P component", error * MiniPID.P);
        telemetry.addData("I component", error * MiniPID.I);
        telemetry.addData("D component", error * MiniPID.D);
        telemetry.update();
    }

    private void initMotors() {
        motors.forEach((name, motor) -> {
            // Reset encoders
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Set motor directions to drive forwards
            switch(name) {
                case FRONTLEFT:
                case BACKLEFT:
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    break;
                case FRONTRIGHT:
                case BACKRIGHT:
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    break;
            }
        });
        // setMotorPower(0);
    }
}
