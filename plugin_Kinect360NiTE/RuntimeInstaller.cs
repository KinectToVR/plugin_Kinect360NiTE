using System;
using Amethyst.Contract;
using Avalonia.Controls;
using Avalonia.Media;

namespace plugin_Kinect360NiTE;

internal class SetupData : ICoreSetupData
{
    public object PluginIcon
    {
        get => new PathIcon
        {
            Data = Geometry.Parse(
                "M45.26,18.3V15.93H69.51V1.1H0V16H24.25v2.37H0v5.25H69.51V18.3ZM9.36,13.19A4.63,4.63,0,0,1,4.65,8.45a4.61,4.61,0,0,1,4.6-4.67,4.71,4.71,0,1,1,.11,9.41Z")
        };
    }

    public string GroupName
    {
        get => "kinect";
    }

    public Type PluginType
    {
        get => typeof(ITrackingDevice);
    }
}
