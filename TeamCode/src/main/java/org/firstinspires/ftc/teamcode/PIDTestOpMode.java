package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.EnumMap;
import java.util.Locale;

@SuppressWarnings("unused")
@Config
@TeleOp
public class PIDTestOpMode extends OpMode {

    public enum MotorNames {
        FRONTLEFT, FRONTRIGHT, BACKRIGHT, BACKLEFT
    }

    public enum State {
        IDLE, DISTANCE, DISPLAY, ROTATION
    }

    public EnumMap<MotorNames, DcMotor> motors;

    State state;
    public double error;
    public double output;

    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    Orientation angles;

    public double currentAngle;
    public static double targetDistance = 50; // distance from sensor (50mm)
    public static double targetAngle = 90; // distance from sensor (50mm)
    private DistanceSensor sensorDistance;
    private BNO055IMU imu;
    private MiniPID pid;

    @Override
    public void init() {
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
        initImu();
        fl = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        fr = hardwareMap.get(DcMotor.class, "BACKLEFT");
        bl = hardwareMap.get(DcMotor.class, "FRONTRIGHT");
        br = hardwareMap.get(DcMotor.class, "FRONTLEFT");

        state = State.IDLE;
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid = new MiniPID(0.001, 0.02, 0.0);
        pid.setSetpoint(targetDistance);
        pid.setOutputLimits(-.5,.5);
        pid.setOutputRampRate(0.1);
    }

    private void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        double distance = sensorDistance.getDistance(DistanceUnit.MM);
        if (gamepad1.a) {
            state = State.DISTANCE;
        } else if (gamepad1.x) {
            state = State.DISPLAY;
        } else if (gamepad1.b) {
            state = State.ROTATION;
        } else if (gamepad1.x) {
            state = State.IDLE;
        }

        switch (state) {
            case DISTANCE:
                output = pid.getOutput(distance, targetDistance);
                fl.setPower(output);
                fr.setPower(output);
                bl.setPower(output);
                br.setPower(output);
                double error = targetDistance - distance;
                telemetry.addData("error", error);
                telemetry.addData("distance", distance);
                break;
            case DISPLAY:
                output = pid.getOutput(distance, targetDistance);
                error = targetDistance - distance;
                telemetry.addData("error", error);
                telemetry.addData("distance", distance);
                break;
            case ROTATION:
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                output = pid.getOutput(angles.firstAngle, targetAngle);
                fl.setPower(output);
                fr.setPower(-output);
                bl.setPower(output);
                br.setPower(-output);
                telemetry.addData("heading", angles.firstAngle);
                break;
        }
        telemetry.addData("output", output);

        telemetry.addData("P component", MiniPID.P);
        telemetry.addData("I component", MiniPID.I);
        telemetry.addData("D component", MiniPID.D);
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

// 0.02, 0, 0.01
