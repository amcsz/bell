package org.firstinspires.ftc.teamcode;

import java.util.Calendar;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;



@Autonomous(name = "Bell", group = "Autonomous")
public class Bell extends OpMode {

    private final double BELL_SPEED = 1;
    private final double TIME_PER_RUN = 30;
    private final double BASE_POWER = 0.15;
    
    
    // set to false for production
    private final boolean enableLateRuns = false;

    ArrayList<ArrayList<Integer>> times = new ArrayList<>(
        Arrays.asList(
            new ArrayList<>(Arrays.asList(10, 50, 0, -1)),
            new ArrayList<>(Arrays.asList(10, 33, 0, 1)),
            new ArrayList<>(Arrays.asList(10, 1, 0, -1)),
            new ArrayList<>(Arrays.asList(10, 0, 0, 1))
        )
    );


    private DcMotor l;
    private DcMotor r;
    private DcMotor b;

    private DistanceSensor E1;
    private DistanceSensor DF;
    private DistanceSensor DB;

    private LED led;

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

        led = hardwareMap.get(LED.class, "L1");

        E1 = hardwareMap.get(DistanceSensor.class, "E1");
        DF = hardwareMap.get(DistanceSensor.class, "DF");
        DB = hardwareMap.get(DistanceSensor.class, "DB");

        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        b.setDirection(DcMotorSimple.Direction.FORWARD);
        led.off();
    }

    @Override
    public void loop() {
        Calendar now = Calendar.getInstance();
        int hour = now.get(Calendar.HOUR_OF_DAY);
        int minute = now.get(Calendar.MINUTE);
        int second = now.get(Calendar.SECOND);

        if (E1.getDistance(DistanceUnit.CM) > 10 || E1.getDistance(DistanceUnit.CM) < 3) {
            led.on();
            requestOpModeStop();
        }

        switch (state) {
            case WAITING:
                for (int i = times.size() - 1; i >= 0; i--) {
                    ArrayList<Integer> time = times.get(i);
                    if (hasReachedTime(time)) {
                           state = State.RUNNING_ACTION;
                           direction = time.get(3);
                           times.remove(i);
                           break;
                    }
                }
                telemetry.addData("State", state);
                telemetry.addData("Time", String.format("%02d:%02d:%02d", hour, minute, second));
                telemetry.addData("front sensor", DF.getDistance(DistanceUnit.CM));
                telemetry.addData("back sensor", DB.getDistance(DistanceUnit.CM));
                telemetry.update();
                l.setPower(0);
                r.setPower(0);
                b.setPower(0);
                break;

            case RUNNING_ACTION:
                if (!isMoving) {
                    moveTimer.reset();
                    isMoving = true;
                }

                if (moveTimer.seconds() < TIME_PER_RUN) {
                    double leftPower = BASE_POWER * direction;
                    double rightPower = BASE_POWER * direction;
                    
                    double front_dist = DF.getDistance(DistanceUnit.CM);
                    double back_dist = DB.getDistance(DistanceUnit.CM);
                    double diff = Math.abs(Math.abs(front_dist - back_dist) - 0.2);

                    if (front_dist < back_dist) {
                        if (direction == 1) {
                            rightPower += diff / 5;
                        } else {
                            leftPower -= diff / 5;
                        }
                    } else {
                        if (direction == 1) {
                            leftPower += diff / 5;
                        } else {
                            rightPower -= diff / 5;
                        }
                    }
                    
                    if (((int)moveTimer.seconds() % 10) < 8) {
                        telemetry.addData("pulsing", "true");
                        if (direction == -1) {
                            leftPower -= 0.08;
                        } else if (direction == 1) {
                            leftPower += 0.08;
                        }
                    }

                    leftPower = Math.min(Math.max(leftPower, -1), 1);
                    rightPower = Math.min(Math.max(rightPower, -1), 1);
                    l.setPower(leftPower);
                    r.setPower(rightPower);
                    b.setPower(BELL_SPEED);
                    
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
                    telemetry.addData("front sensor", DF.getDistance(DistanceUnit.CM));
                    telemetry.addData("back sensor", DB.getDistance(DistanceUnit.CM));
                    telemetry.addData("direction", direction);
                    telemetry.addData("timer", moveTimer.seconds());
                    telemetry.update();
                } else {
                    ran[numberRan] = true;
                    numberRan++;
                    if (numberRan == 4) {
                        state = State.DONE;
                    } else {
                        state = State.WAITING;
                    }
                    isMoving = false;
                }
                break;

            case DONE:
                requestOpModeStop();
                break;
        }
    }

    private boolean hasReachedTime(ArrayList<Integer> target) {
        int hTarget = target.get(0);
        int mTarget = target.get(1);
        int sTarget = target.get(2);
        Calendar now = Calendar.getInstance();
        int hNow = now.get(Calendar.HOUR_OF_DAY);
        int mNow = now.get(Calendar.MINUTE);
        int sNow = now.get(Calendar.SECOND);
        boolean res = (hNow > hTarget) ||
                   (hNow == hTarget &&
                       (mNow > mTarget ||
                       (mNow == mTarget && sNow >= sTarget)));
        if (enableLateRuns) {
            return res;
        }
        return res &&
               (
                   (hNow < hTarget) ||
                   (hNow == hTarget &&
                       (mNow < mTarget + 1 ||
                       (mNow == mTarget + 1 && sNow < sTarget)))
               ); 
    }
}

