using System.Diagnostics;
using Amethyst.Contract;

namespace plugin_Kinect360NiTE;

public class GestureDetector
{
    private bool Value { get; set; }
    private bool ValueBlock { get; set; }
    private Stopwatch Timer { get; set; } = new();

    public bool Update(bool value)
    {
        // ReSharper disable once ConvertIfStatementToSwitchStatement
        if (!Value && value)
        {
            //Console.WriteLine("Restarting gesture timer...");
            ValueBlock = false;
            Timer.Restart();
            Value = true;
            return false;
        }

        if (!Value && !value)
        {
            //Console.WriteLine("Resetting gesture timer...");
            ValueBlock = false;
            Timer.Reset();
            return false;
        }

        Value = value;

        switch (Timer.ElapsedMilliseconds)
        {
            case >= 1000 when !ValueBlock:
                //Console.Write("Gesture detected! ");
                Kinect360.HostStatic?.PlayAppSound(SoundType.Focus);
                ValueBlock = true;
                return true;
            case >= 3000 when ValueBlock:
                //Console.Write("Restarting timer...");
                Kinect360.HostStatic?.PlayAppSound(SoundType.Focus);
                ValueBlock = false;
                Timer.Restart();
                return true;
            default:
                //Console.WriteLine("Gesture detected! Waiting for the timer...");
                return false;
        }
    }
}
