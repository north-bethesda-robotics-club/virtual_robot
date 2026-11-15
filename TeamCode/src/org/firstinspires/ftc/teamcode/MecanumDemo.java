package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "mecanum bot demo", group = "MecanumBot")
public class MecanumDemo extends LinearOpMode {

    public void CheckShootNectar(double triggerValue) {
        DcMotorEx m7 = (DcMotorEx) hardwareMap.dcMotor.get("shoot_nectar_motor");
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double shootVelocity = triggerValue;

        if (triggerValue > 0 && triggerValue <= 0.5)
        {
            shootVelocity = 1500;
        }
        if (triggerValue > 0.5) {
            shootVelocity = 3000;
        }
        m7.setVelocity(shootVelocity);
    }
    public void CheckShootPollen(double triggerValue2) {
        DcMotorEx m6 = (DcMotorEx) hardwareMap.dcMotor.get("shoot_pollen_motor");
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double shootVelocity2 = triggerValue2;

        if (triggerValue2 > 0 && triggerValue2 <= 0.5)
        {
            shootVelocity2 = 1250;
        }
        if (triggerValue2 > 0.5) {
            shootVelocity2 = 2500;
        }
        m6.setVelocity(shootVelocity2);
    }

    public void runOpMode(){
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor m5 = hardwareMap.dcMotor.get("turret_motor");
        DcMotorEx m6 = (DcMotorEx) hardwareMap.dcMotor.get("shoot_pollen_motor");
        DcMotorEx m7 = (DcMotorEx) hardwareMap.dcMotor.get("shoot_nectar_motor");
        DcMotor m8 = hardwareMap.dcMotor.get("intake_motor");
        CRServo pollen_transfer = hardwareMap.crservo.get("pollen_transfer");

        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
//        OctoQuad octoQuad = hardwareMap.get(OctoQuad.class, "octoquad");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        VectorF oldVel = new VectorF(0,0);
        double oldSecs = getRuntime();
        double maxVel = 0;
        double maxAccel = 0;
        double minAccel = 0;

        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            //double pa = gamepad1.left_trigger - gamepad1.right_bumper;
            double pa = 0.0;
            boolean dpadL = gamepad1.dpad_left;
            boolean dpadR = gamepad1.dpad_right;
            double RightTrigger = gamepad1.right_trigger;
            double LeftTrigger = gamepad1.left_trigger;
            pa = gamepad1.right_stick_x;

            boolean left_bumper = gamepad1.left_bumper;
            if (left_bumper) {
                m8.setPower(1);
                pollen_transfer.setPower(1);
            }
            if (!left_bumper) {
                m8.setPower(0);
                pollen_transfer.setPower(0);
            }

            //if (Math.abs(pa) < 0.05) pa = 0;
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);
//THIS IS FOR THE TURRET AIMING AND SHOOTING ON CONTROLLER (M5) {
            double turretPowerFactor = 0.5;
            double turretPower = 0.0;
            if (dpadL) {
                turretPower += turretPowerFactor;
            }
            if (dpadR) {
                turretPower -= turretPowerFactor;
            }
            m5.setPower(turretPower);

            CheckShootNectar(LeftTrigger);
            CheckShootPollen(RightTrigger);


            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double secs = getRuntime();
            VectorF vel = new VectorF((float)pinpoint.getVelX(DistanceUnit.INCH),
                    (float)pinpoint.getVelY(DistanceUnit.INCH));
            VectorF accel = vel.subtracted(oldVel).multiplied(1.0f/((float)secs - (float)oldSecs));

            if (vel.get(0) > maxVel) maxVel = vel.get(0);
            if (accel.get(0) > maxAccel) maxAccel = accel.get(0);
            if (accel.get(0) < minAccel) minAccel = accel.get(0);

            oldVel = vel;
            oldSecs = secs;

            telemetry.addData("maxVel", maxVel);
            telemetry.addData("maxAccel", maxAccel);
            telemetry.addData("minAccel", minAccel);

            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
//            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
//            telemetry.addData("Octoquad", "%d %d %d %d", octoQuad.readSinglePosition(0),
//                    octoQuad.readSinglePosition(1), octoQuad.readSinglePosition(2),
//                    octoQuad.readSinglePosition(3));
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setVelocity(0);
        m7.setVelocity(0);
    }
}
