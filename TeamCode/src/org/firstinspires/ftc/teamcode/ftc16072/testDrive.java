package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testDrive extends LinearOpMode {

    private DcMotor left_motor, right_motor;

    @Override
    public void runOpMode(){
        left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");

        left_motor.setDirection(DcMotor.Direction.REVERSE);

        double y;
        double x;
        double leftPower;
        double rightPower;

        waitForStart();

        while(opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.right_stick_x / 2;

            leftPower = y + x;
            rightPower = y - x;
            if (leftPower > 1) {
                rightPower -= 1 - (2 - leftPower);
                leftPower = 1;
            }
            if (rightPower > 1) {
                leftPower -= 1 - (2 - rightPower);
                rightPower = 1;
            }
            if (leftPower < -1) {
                rightPower += 1 - (2 + leftPower);
                leftPower = -1;
            }
            if (rightPower < -1) {
                leftPower += 1 - (2 + rightPower);
                rightPower = -1;
            }
            left_motor.setPower(leftPower);
            right_motor.setPower(rightPower);

            telemetry.addData("Left Power:", left_motor.getPower());
            telemetry.addData("Right Power:", right_motor.getPower());
            telemetry.update();
        }
    }
}
