package org.firstinspires.ftc.teamcode.util;

/**
 * The class Timer canceller.
 */
public class TimerCanceller implements Canceller
{
    private long ms, start;

    private long timeRemaining() {
        return start + ms - System.currentTimeMillis();
    }

    /**
     * Instantiates a new Timer canceller.
     *
     * @param ms the ms
     */
    public TimerCanceller(long ms)
    {
        reset(ms);
    }

    public void reset(long ms)
    {
        this.ms = ms;
        start = System.currentTimeMillis();
    }

    public void reset()
    {
        reset(this.ms);
    }

    //Returns if the time requirement is met
    public boolean isConditionMet()
    {
        return timeRemaining() <= 0;
    }

    //Waits for timer to finish
    public void waitForCanceller()
    {
        long remaining = timeRemaining();
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}