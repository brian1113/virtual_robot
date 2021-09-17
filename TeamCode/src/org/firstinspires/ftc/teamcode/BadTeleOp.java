package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.math.BigDecimal;
import java.math.RoundingMode;


@TeleOp
public class BadTeleOp extends LinearOpMode {

    private BNO055IMU imu;
    private DcMotor front_left_motor, front_right_motor, back_left_motor, back_right_motor;
    boolean accelerating = false;

    @Override
    public void runOpMode(){
        initializeIMU();
        initializeHardware();
        addTelemetryData();
        waitForStart();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y  ;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx), 1);

            double v1 = (y+x+rx)/denominator;
            double v2 = (y-x-rx)/denominator;
            double v3 = (y-x+rx)/denominator;
            double v4 = (y+x-rx)/denominator;

            front_left_motor.setPower(v1);
            front_right_motor.setPower(v2);
            back_left_motor.setPower(v3);
            back_right_motor.setPower(v4);
            addTelemetryData();
            telemetry.update();
        }
    }

    private void addTelemetryData(){
        telemetry.addData("Front left power", round(front_left_motor.getPower()));
        telemetry.addData("Front right power", round(front_right_motor.getPower()));
        telemetry.addData("Back left power", round(back_left_motor.getPower()));
        telemetry.addData("Back right power", round(back_right_motor.getPower()));
    }

    private void initializeIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        telemetry.addData("Status", "Imu Calibrated");
        telemetry.update();
    }

    private void initializeHardware(){
        telemetry.addData("Status", "Initializing hardware");
        telemetry.update();
        //set motors
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        //specify motor directions
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        //set auto brake
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        telemetry.addData("Status", "Done Initializing");
        telemetry.update();
    }

    private static double round(double value) {
        return round(value, 4);
    }

    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
