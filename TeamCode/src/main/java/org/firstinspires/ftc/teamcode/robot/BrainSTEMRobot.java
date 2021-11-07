package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.util.MinMaxAverage;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

public class BrainSTEMRobot implements Component {
    //Various components of the  robot
    public CarouselSpin carouselSpin;
    public Collector collector;
    public DepositorLift depositorLift;
    public SampleTankDrive drive;
    public Turret turret;

    //Instance of linear opmode to use for hwMap
    private LinearOpMode opMode;

    private Thread updateThread;
    private boolean started = false;

    //List of components to be initialized
    private ArrayList<Component> components;

    /**
     * Instantiates a new robot.
     *
     * @param opMode the op mode
     */
    public BrainSTEMRobot(LinearOpMode opMode)
    {
        this.opMode = opMode;

        //Get instance of hardware map and telemetry
        HardwareMap map = opMode.hardwareMap;

        drive = new SampleTankDrive(opMode.hardwareMap, this);

        components = new ArrayList<>();

        //Initialize robot components
//        carouselSpin = new CarouselSpin(map);
//        collector = new Collector(map);
//        depositorLift = new DepositorLift(map, opMode.telemetry);
//        turret = new Turret(map);

        //Add all components to an array list so they can be easily initialized
//        components.add(carouselSpin);
//        components.add(collector);
//        components.add(depositorLift);
//        components.add(turret);
    }

    @Override
    public void update()
    {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : allHubs) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        MinMaxAverage mma = new MinMaxAverage();
        ElapsedTime time = new ElapsedTime();
        while (started) {
            for (LynxModule m: allHubs) {
                m.getBulkData();
            }

            for (Component component : components) {
                if (!opMode.isStopRequested()) {
                    component.update();
                } else {
                    stop();
                }
            }

            double currentTime = time.milliseconds();
            time.reset();
            mma.update(currentTime);

            opMode.telemetry.addLine("Loop time: " + currentTime);
            opMode.telemetry.addLine(mma.toString());
            opMode.telemetry.update();
        }
    }

    public void stop()
    {
        started = false;
    }

    @Override
    public void reset() {
        for (Component component : components)
            component.reset();
    }

    public String test()
    {
        String failures = "";
        for (Component component : components)
            failures += component.test();
        return failures;
    }
}