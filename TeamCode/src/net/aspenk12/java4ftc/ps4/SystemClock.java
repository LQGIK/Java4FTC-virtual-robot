package net.aspenk12.java4ftc.ps4;

public class SystemClock implements Clock {
    private long startTime;
    public SystemClock(){
        startTime = System.currentTimeMillis();
    }

    public long getCurrentTime(){
        return System.currentTimeMillis();
    }
    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

}
