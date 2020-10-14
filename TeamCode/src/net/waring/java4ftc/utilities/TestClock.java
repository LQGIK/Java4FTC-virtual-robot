package net.waring.java4ftc.utilities;

public class TestClock implements Clock {

    private double time;
    private double timeStep;
    private long startTime;

    public TestClock(){
        startTime = System.currentTimeMillis();
        time = 0;
        timeStep = 1;
    }

    public long getCurrentTime(){
        return System.currentTimeMillis();
    }

    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

    public double getTime(){
        return time;
    }

    public void setTimeStep(double millis){
        timeStep = millis;
    }

    public void incrementTime() { time += timeStep; }

}
