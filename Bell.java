package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Calendar;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Calendar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Bell", group = "Autonomous")
public class Bell extends OpMode {

    private final double BASE_POWER = 0.3;
    private final double BELL_SPEED = 1;
    private final double TIME_PER_RUN = 10;
    

    private DcMotor l;
    private DcMotor r;
    private DcMotor b;
    private IMU imu;
    
    private RevColorSensorV3 c1;
    private DistanceSensor d1;
    
    private boolean[] ran = new boolean[4];
    private int numberRan = 0;
    
    private ElapsedTime moveTimer = new ElapsedTime();
    private boolean isMoving = false;
    private int direction = 1;
    
    enum State { WAITING, RUNNING_ACTION, DONE }

    State state = State.WAITING;

    @Override
    public void init() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        b = hardwareMap.dcMotor.get("b");
        
        imu = hardwareMap.get(IMU.class, "imu");
        
        c1 = hardwareMap.get(RevColorSensorV3.class, "C1");
        d1 = hardwareMap.get(DistanceSensor.class, "D1");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    

    @Override
    public void loop() {
        Calendar now = Calendar.getInstance();
        int hour = now.get(Calendar.HOUR_OF_DAY);
        int minute = now.get(Calendar.MINUTE);
        int second = now.get(Calendar.SECOND);
        telemetry.addData("State", state);
        telemetry.addData("Time", String.format("%02d:%02d:%02d", hour, minute, second));
        telemetry.addData("numberRan", numberRan);
        telemetry.addData("front sensor", ((DistanceSensor)c1).getDistance(DistanceUnit.CM) - 0.63);
        telemetry.addData("back sensor", d1.getDistance(DistanceUnit.CM) - 1.85);
        telemetry.update();
        switch (state) {
            case WAITING:
                if (hasReachedTime(hour, minute, second, 9, 38, 0) && (!ran[0])
                || hasReachedTime(hour, minute, second, 9, 38, 30) && (!ran[1])
                || hasReachedTime(hour, minute, second, 9, 39, 0) && (!ran[2])
                || hasReachedTime(hour, minute, second, 9, 39, 30) && (!ran[3])) {
                    state = State.RUNNING_ACTION;
                }
                break;

            case RUNNING_ACTION:
                if (!isMoving) {
                    moveTimer.reset();
                    isMoving = true;
                }
            
                if (moveTimer.seconds() < TIME_PER_RUN) {
                    double leftPower = BASE_POWER * direction, rightPower = BASE_POWER * direction;
                    if (direction == -1) {
                        leftPower += 0.1;
                        rightPower += 0.1;
                    }
                    double front_dist = ((DistanceSensor)c1).getDistance(DistanceUnit.CM) - 0.63;
                    double back_dist = d1.getDistance(DistanceUnit.CM) - 1.85;
                    if (front_dist < back_dist) {
                        if (direction == 1) {
                            rightPower += (back_dist / front_dist) / 5;
                        } else {
                            leftPower -= (back_dist / front_dist) / 5;
                        }
                    } else {
                        if (direction == 1) {
                            leftPower += (front_dist / back_dist) / 5;
                        } else {
                            rightPower -= (front_dist / back_dist) / 5;
                        }
                        
                    }
                    
                    if (direction == -1) {
                        leftPower -= 0.025;
                    } else {
                        leftPower -= 0.05;
                        rightPower -= 0.05;
                    }

                    leftPower = Math.min(Math.max(leftPower, -1), 1);
                    rightPower = Math.min(Math.max(rightPower, -1), 1);
            
                    l.setPower(leftPower);
                    r.setPower(rightPower);
                    telemetry.addData("left", leftPower);
                    telemetry.addData("right", rightPower);
                    if (direction == 1) {
                        if (leftPower > rightPower) {
                            telemetry.addData("dir", "moving left");
                        } else {
                            telemetry.addData("dir", "moving right");
                        }
                    } else {
                        if (leftPower < rightPower) {
                            telemetry.addData("dir", "moving left");
                        } else {
                            telemetry.addData("dir", "moving right");
                        }
                    }
                    telemetry.update();
                    b.setPower(-1 * BELL_SPEED);
                    
                } else {
                    l.setPower(0);
                    r.setPower(0);
                    b.setPower(0);
                    ran[numberRan] = true;
                    numberRan++;
                    direction *= -1;
                    if (numberRan == 4) {
                        state = State.DONE;
                    } else {
                        state = State.WAITING;
                    }
                    isMoving = false;
                }
                break;
            case DONE:
                break;
        }
    }
    
    private boolean hasReachedTime(int hNow, int mNow, int sNow, int hTarget, int mTarget, int sTarget) {
        return hNow > hTarget ||
               (hNow == hTarget && (mNow > mTarget || (mNow == mTarget && sNow >= sTarget)));
    }
    }
