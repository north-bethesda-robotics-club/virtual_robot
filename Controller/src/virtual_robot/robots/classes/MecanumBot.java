package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.geometry.Point2D;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Polygon;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, a Servo-controlled arm on the back, and three dead-wheel encoder pods
 *
 * MecanumBot is the controller class for the "mecanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "Mecanum Bot", filename = "mecanum_bot")
public class MecanumBot extends MecanumPhysicsBase {

    private ServoImpl servo = null;

    private MotorType encoderMotorType;
    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    // Additional mechanism motors. Power and encoder counts are simulated, and the intake and
    // shooters are animated on screen (see updateMechanismDisplay):
    //   intake_motor       - roller at the BACK of the robot. Positive power pulls in, negative spits out.
    //   shoot_nectar_motor - shooter at the FRONT LEFT. Positive power fires orange balls.
    //   shoot_pollen_motor - shooter at the FRONT RIGHT. Positive power fires yellow balls.
    //   turret_motor       - simulated, not drawn.
    private static final String[] MECHANISM_MOTOR_NAMES = {"intake_motor", "turret_motor", "shoot_pollen_motor", "shoot_nectar_motor"};
    private DcMotorExImpl[] mechanismMotors = new DcMotorExImpl[MECHANISM_MOTOR_NAMES.length];
    private DcMotorExImpl intakeMotor, leftShooterMotor, rightShooterMotor;

    @FXML Rectangle intakeRoller;
    @FXML Polyline intakeChevron0, intakeChevron1, intakeChevron2;
    @FXML Polygon leftMuzzle, rightMuzzle;
    @FXML Circle leftFlywheel, rightFlywheel;
    private Polyline[] intakeChevrons;

    private static final Color NECTAR_COLOR = Color.web("#ff7a00");
    private static final Color POLLEN_COLOR = Color.web("#ffd600");
    private static final Color ROLLER_IDLE = Color.web("#2b2b2b");
    private static final Color INTAKE_IN_COLOR = Color.LIMEGREEN;
    private static final Color INTAKE_OUT_COLOR = Color.TOMATO;
    private static final double SHOOT_THRESHOLD = 0.15;   // minimum shooter speed (fraction of max) that fires balls
    private static final double BALL_RADIUS_INCHES = 2.0;

    /** A ball fired by a shooter, drawn on the field (not on the robot). */
    private static class Projectile {
        final Circle circle;
        final double vx, vy, range;   // pixels/sec in field-pane coordinates; range in pixels
        double traveled = 0;
        Projectile(Circle circle, double vx, double vy, double range) {
            this.circle = circle; this.vx = vx; this.vy = vy; this.range = range;
        }
    }
    private final List<Projectile> projectiles = new ArrayList<>();
    private double intakePhase = 0, leftFireTimer = Double.MAX_VALUE, rightFireTimer = Double.MAX_VALUE;
    private long lastDisplayNanos = 0;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    //Dimensions in inches for encoder wheels.
    //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
    //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double LEFT_ENCODER_X = -6.0;
    private final double RIGHT_ENCODER_X = 6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = 0.0;

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    public MecanumBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");
        for (int i = 0; i < MECHANISM_MOTOR_NAMES.length; i++) {
            mechanismMotors[i] = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, MECHANISM_MOTOR_NAMES[i]);
        }
        intakeMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "intake_motor");
        leftShooterMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "shoot_nectar_motor");
        rightShooterMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "shoot_pollen_motor");
        intakeChevrons = new Polyline[]{intakeChevron0, intakeChevron1, intakeChevron2};

        octoQuad.setEncoder(4, leftEncoder);
        octoQuad.setEncoder(5, rightEncoder);
        octoQuad.setEncoder(6, xEncoder);

        //Dimensions in pixels
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        hardwareMap.setActive(false);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        encoderMotorType = MotorType.Neverest40;
        hardwareMap.put("back_servo", new ServoImpl());
        String[] encoderNames = new String[] {"enc_right", "enc_left", "enc_x"};
        for (int i=0; i<3; i++){
            hardwareMap.put(encoderNames[i], new DeadWheelEncoder(MOTOR_TYPE, motorController1, i));
        }
        // Mechanism motors use motorController1 ports 3-6 (ports 0-2 are the dead wheel encoders).
        for (int i = 0; i < MECHANISM_MOTOR_NAMES.length; i++) {
            hardwareMap.put(MECHANISM_MOTOR_NAMES[i], new DcMotorExImpl(MotorType.Neverest40, motorController1, 3 + i));
        }
    }

    public synchronized void updateStateAndSensors(double millis){

        //Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        //Compute new pose and update various sensors
        super.updateStateAndSensors(millis);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

        //Advance mechanism motors so their speed and encoder counts respond to power
        for (DcMotorExImpl m : mechanismMotors) m.update(millis);

    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
        updateMechanismDisplay();
    }

    /** Signed motor speed as a fraction of max (-1 to 1), using the OpMode's direction setting. */
    private static double speedOf(DcMotorExImpl motor) {
        return motor.getVelocity() / motor.MOTOR_TYPE.MAX_TICKS_PER_SECOND;
    }

    /** Animate the intake and shooters. Runs on the JavaFX thread. */
    private void updateMechanismDisplay() {
        long now = System.nanoTime();
        double dt = lastDisplayNanos == 0 ? 0 : Math.min(0.1, (now - lastDisplayNanos) / 1e9);
        lastDisplayNanos = now;

        // Intake: roller lights up, chevrons flow toward the robot (in) or away from it (out).
        double intake = speedOf(intakeMotor);
        double intakeMag = Math.min(1.0, Math.abs(intake));
        Color intakeColor = intake >= 0 ? INTAKE_IN_COLOR : INTAKE_OUT_COLOR;
        intakeRoller.setFill(ROLLER_IDLE.interpolate(intakeColor, Math.min(1.0, intakeMag * 2)));
        intakePhase = (intakePhase + 1.5 * intake * dt) % 1.0;
        if (intakePhase < 0) intakePhase += 1.0;
        for (int i = 0; i < intakeChevrons.length; i++) {
            double d = (i / (double) intakeChevrons.length + 1.0 - intakePhase) % 1.0; // 0 = at roller, 1 = far
            Polyline chevron = intakeChevrons[i];
            chevron.setTranslateX(37.5);
            chevron.setTranslateY(86.0 + 30.0 * d);
            chevron.setRotate(intake >= 0 ? 0 : 180);
            chevron.setStroke(intakeColor);
            chevron.setOpacity(intakeMag < 0.05 ? 0 : Math.min(1.0, intakeMag * 1.5) * Math.sin(Math.PI * d));
        }

        double left = speedOf(leftShooterMotor);
        double right = speedOf(rightShooterMotor);
        leftFireTimer = updateShooter(left, leftFlywheel, leftMuzzle, 21.0, NECTAR_COLOR, leftFireTimer, dt);
        rightFireTimer = updateShooter(right, rightFlywheel, rightMuzzle, 54.0, POLLEN_COLOR, rightFireTimer, dt);

        // Move balls already in flight, fading them out near the end of their range.
        Iterator<Projectile> it = projectiles.iterator();
        while (it.hasNext()) {
            Projectile p = it.next();
            p.circle.setCenterX(p.circle.getCenterX() + p.vx * dt);
            p.circle.setCenterY(p.circle.getCenterY() + p.vy * dt);
            p.traveled += Math.hypot(p.vx, p.vy) * dt;
            double cx = p.circle.getCenterX(), cy = p.circle.getCenterY();
            boolean offField = cx < 0 || cy < 0 || cx > VirtualField.FIELD_WIDTH || cy > VirtualField.FIELD_WIDTH;
            if (p.traveled >= p.range || offField) {
                fieldPane.getChildren().remove(p.circle);
                it.remove();
            } else {
                p.circle.setOpacity(Math.min(1.0, 3.0 * (1.0 - p.traveled / p.range)));
            }
        }
    }

    /**
     * Spin the flywheel, show the muzzle flash, and fire balls at a rate that grows with speed.
     * @param muzzleX robot-local x of the shooter (the robot front edge is y = 0)
     * @return updated fire timer
     */
    private double updateShooter(double speed, Circle flywheel, Polygon muzzle, double muzzleX,
                                 Color color, double fireTimer, double dt) {
        flywheel.setStrokeDashOffset(flywheel.getStrokeDashOffset() - 150.0 * speed * dt);
        boolean firing = speed > SHOOT_THRESHOLD;
        muzzle.setOpacity(firing ? Math.min(1.0, speed) : 0);
        if (!firing) return Double.MAX_VALUE;   // fire immediately once the shooter spins up

        double interval = 0.35 / speed;
        fireTimer = fireTimer == Double.MAX_VALUE ? interval : fireTimer + dt;
        if (fireTimer < interval) return fireTimer;

        // Convert the muzzle position and the robot's forward direction to field-pane coordinates.
        Point2D start = displayGroup.localToParent(muzzleX, -4);
        Point2D ahead = displayGroup.localToParent(muzzleX, -14);
        Point2D dir = ahead.subtract(start).normalize();
        double fieldWidth = VirtualField.FIELD_WIDTH;
        double ballSpeed = speed * fieldWidth * 0.9;            // pixels per second
        double range = fieldWidth * (0.25 + 0.5 * speed);       // pixels

        Circle ball = new Circle(start.getX(), start.getY(), BALL_RADIUS_INCHES * botWidth / 18.0, color);
        ball.setStroke(Color.BLACK);
        ball.setStrokeWidth(1.0);
        ball.setMouseTransparent(true);
        fieldPane.getChildren().add(ball);
        projectiles.add(new Projectile(ball, dir.getX() * ballSpeed, dir.getY() * ballSpeed, range));
        return 0;
    }

    /** Remove balls in flight and return the intake/shooter graphics to idle. Runs on the JavaFX thread. */
    private synchronized void resetMechanismDisplay() {
        for (Projectile p : projectiles) fieldPane.getChildren().remove(p.circle);
        projectiles.clear();
        lastDisplayNanos = 0;
        leftFireTimer = rightFireTimer = Double.MAX_VALUE;
        if (intakeRoller == null) return;
        intakeRoller.setFill(ROLLER_IDLE);
        for (Polyline chevron : intakeChevrons) chevron.setOpacity(0);
        leftMuzzle.setOpacity(0);
        rightMuzzle.setOpacity(0);
    }

    @Override
    public void removeFromDisplay(Pane fieldPane) {
        resetMechanismDisplay();
        super.removeFromDisplay(fieldPane);
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
        rightEncoder.stopAndReset();
        leftEncoder.stopAndReset();
        xEncoder.stopAndReset();
        for (DcMotorExImpl m : mechanismMotors) m.stopAndReset();
        Platform.runLater(this::resetMechanismDisplay);
    }


}
