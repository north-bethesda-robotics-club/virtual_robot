package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Robot-centric mecanum TeleOp.
 *
 * This file is in TeamCode/robot, so it runs in the simulator AND gets deployed
 * to the Control Hub. Only use FTC SDK classes here (com.qualcomm.*, org.firstinspires.*),
 * never virtual_robot.* classes.
 *
 * Controls (gamepad1):
 *   left stick  - drive forward/back and strafe left/right
 *   right stick - turn
 */
@TeleOp(name = "Basic Mecanum TeleOp", group = "Team")
public class BasicMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Left side motors face the opposite way, so reverse them.
        // On the real robot, check this: if it drives backwards or spins, flip these.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double fl = forward + strafe + turn;
            double bl = forward - strafe + turn;
            double fr = forward - strafe - turn;
            double br = forward + strafe - turn;

            // Scale down so no motor power is above 1.0.
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br))));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            telemetry.addData("Forward / Strafe / Turn", "%.2f / %.2f / %.2f", forward, strafe, turn);
            telemetry.update();
        }
    }
}
