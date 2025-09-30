using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel.Composition;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Runtime.InteropServices;
using Amethyst.Contract;
using Avalonia.Controls;
using Avalonia.Media.Imaging;
using NiTEWrapper;
using OpenNIWrapper;
using SkiaSharp;
using Quaternion = System.Numerics.Quaternion;

namespace plugin_Kinect360NiTE;

[Export(typeof(ITrackingDevice))]
[ExportMetadata("Name", "Xbox 360 Kinect (NiTE)")]
[ExportMetadata("Guid", "K2VRTEAM-AME2-APII-DVCE-NITEKINECTV1")]
[ExportMetadata("Publisher", "K2VR Team")]
[ExportMetadata("Version", "1.0.0.1")]
[ExportMetadata("Website", "https://github.com/KinectToVR/plugin_Kinect360NiTE")]
[ExportMetadata("DependencyLink", "https://docs.k2vr.tech/{0}/360/setup/")]
[ExportMetadata("CoreSetupData", typeof(SetupData))]
public class Kinect360 : ITrackingDevice
{
    [Import(typeof(IAmethystHost))] private IAmethystHost Host { get; set; }

    private bool PluginLoaded { get; set; }
    public bool IsCameraEnabled { get; set; }

    public bool IsPositionFilterBlockingEnabled => false;
    public bool IsPhysicsOverrideEnabled => false;
    public bool IsSelfUpdateEnabled => true;
    public bool IsFlipSupported => true;
    public bool IsAppOrientationSupported => false;
    public object SettingsInterfaceRoot => null;

    public int DeviceStatus => IsInitialized ? 0 : 1;
    public bool IsInitialized { get; set; }
    public bool IsSkeletonTracked { get; set; } // TODO
    public Exception InitException { get; set; }
    public UserTracker BodyTracker { get; set; }
    public Device CurrentDevice { get; set; }
    public VideoStream CurrentSensor { get; set; }
    public VideoStream DepthStream { get; set; }
    public SKBitmap CameraImage { get; set; }
    public static IAmethystHost HostStatic { get; set; }

    private readonly GestureDetector
        _pauseDetectorLeft = new(),
        _pauseDetectorRight = new(),
        _pointDetectorLeft = new(),
        _pointDetectorRight = new();

    public ObservableCollection<TrackedJoint> TrackedJoints { get; } =
        // Prepend all supported joints to the joints list
        new(Enum.GetValues<TrackedJointType>()
            .Where(x => x is not TrackedJointType.JointManual)
            .Select(x => new TrackedJoint
            {
                Name = HostStatic?.RequestLocalizedString($"/JointsEnum/{x.ToString()}") ?? x.ToString(),
                Role = x,
                SupportedInputActions = x switch
                {
                    TrackedJointType.JointHandLeft =>
                    [
                        new KeyInputAction<bool>
                        {
                            Name = "Left Pause", Description = "Left hand pause gesture",
                            Guid = "PauseLeft_One", GetHost = () => HostStatic
                        },
                        new KeyInputAction<bool>
                        {
                            Name = "Left Point", Description = "Left hand point gesture",
                            Guid = "PointLeft_One", GetHost = () => HostStatic
                        }
                    ],
                    TrackedJointType.JointHandRight =>
                    [
                        new KeyInputAction<bool>
                        {
                            Name = "Right Pause", Description = "Right hand pause gesture",
                            Guid = "PauseRight_One", GetHost = () => HostStatic
                        },
                        new KeyInputAction<bool>
                        {
                            Name = "Right Point", Description = "Right hand point gesture",
                            Guid = "PointRight_One", GetHost = () => HostStatic
                        }
                    ],
                    _ => []
                }
            }));

    public string DeviceStatusString
    {
        get => PluginLoaded ?
            DeviceStatus switch
            {
                0 => Host.RequestLocalizedString("/Plugins/KinectOne/Statuses/Success"),
                1 when InitException is null => Host.RequestLocalizedString("/Plugins/KinectOne/Statuses/NotAvailable"),
                1 when InitException is not null => Host.RequestLocalizedString("/Plugins/KinectOne/Statuses/NotAvailable")
                                                    + $"{InitException?.Message ?? "E_UNKNOWN"}",
                _ => $"Undefined: {DeviceStatus}\nE_UNDEFINED\nSomething weird has happened, although we can't tell what."
            } :
            $"Undefined: {DeviceStatus}\nE_UNDEFINED\nSomething weird has happened, although we can't tell what.";
    }

    public Uri ErrorDocsUri
    {
        get => new($"https://docs.k2vr.tech/{Host?.DocsLanguageCode ?? "en"}/one/troubleshooting/");
    }

    public void OnLoad()
    {
        // Backup the plugin host
        HostStatic = Host;
        PluginLoaded = true;

        try
        {
            // Re-generate joint names
            lock (Host.UpdateThreadLock)
            {
                for (var i = 0; i < TrackedJoints.Count; i++)
                {
                    TrackedJoints[i] = TrackedJoints[i].WithName(Host?.RequestLocalizedString(
                        $"/JointsEnum/{TrackedJoints[i].Role.ToString()}") ?? TrackedJoints[i].Role.ToString());

                    foreach (var action in TrackedJoints[i].SupportedInputActions)
                    {
                        action.Name = Host!.RequestLocalizedString($"/InputActions/Names/{action.Guid.Replace("_One", "")}");
                        action.Description = Host.RequestLocalizedString($"/InputActions/Descriptions/{action.Guid.Replace("_One", "")}");

                        action.Image = new Image
                        {
                            Source = Bitmap.DecodeToHeight(File.OpenRead(Path.Join(
                                Directory.GetParent(Assembly.GetExecutingAssembly().Location)!.FullName,
                                "Assets", "Resources", "Icons", $"{(((dynamic)Host).IsDarkMode as bool? ?? false ? "D" : "W")}" +
                                                                $"_{action.Guid.Replace("_One", "")}.png")), 500)
                        };
                    }
                }
            }
        }
        catch (Exception e)
        {
            Host?.Log($"Error setting joint names! Message: {e.Message}", LogSeverity.Error);
        }
    }

    public void Initialize()
    {
        // OpenNI
        {
            OpenNI.Status result;

            try
            {
                result = OpenNI.Initialize();

                OpenNI.OnDeviceConnected += OnDeviceConnected;
                OpenNI.OnDeviceDisconnected += OnDeviceDisconnected;

                var devices = OpenNI.EnumerateDevices(); // TODO

                OnDeviceConnected(devices.FirstOrDefault());
            }
            catch (Exception ex)
            {
                result = OpenNI.Status.Error;
                Host.Log(ex);

                InitException = ex;
                IsInitialized = false;
            }

            switch (result)
            {
                case OpenNI.Status.Ok:
                    Host.Log($"Tried to initialize the Kinect sensor with status: {DeviceStatusString}");
                    break;
                default:
                    Host.Log($"Couldn't initialize the Kinect sensor! Status: {DeviceStatusString} ({result})", LogSeverity.Warning);
                    break;
            }
        }
    }

    public void OnDeviceConnected(DeviceInfo device)
    {
        if (device is null) return;

        var newDevice = device.OpenDevice();

        if (!newDevice.HasSensor(Device.SensorType.Color) ||
            !newDevice.HasSensor(Device.SensorType.Depth) ||
            !newDevice.HasSensor(Device.SensorType.Ir)) return;

        if (CurrentSensor is not null)
        {
            try
            {
                IsInitialized = false;
                CurrentSensor.Stop();
                CurrentSensor.OnNewFrame -= CurrentSensorOnNewFrame;
            }
            catch
            {
                // ignred
            }
        }

        if (DepthStream is not null)
        {
            try
            {
                IsInitialized = false;
                DepthStream.Stop();
            }
            catch
            {
                // ignred
            }
        }

        if (CurrentDevice is not null)
        {
            try
            {
                IsInitialized = false;
                CurrentDevice.Dispose();
            }
            catch
            {
                // ignred
            }
        }

        // If we're okay to go, set up
        CurrentDevice = newDevice;

        // Color
        CurrentSensor = CurrentDevice.CreateVideoStream(Device.SensorType.Color);
        if (CurrentSensor?.Start() is OpenNI.Status.Ok)
            CurrentSensor.OnNewFrame += CurrentSensorOnNewFrame;
        else
            Host.Log("Failed to start video stream.", LogSeverity.Error);

        // Depth
        DepthStream = CurrentDevice.CreateVideoStream(Device.SensorType.Depth);
        if (DepthStream?.Start() is not OpenNI.Status.Ok)
            Host.Log("Failed to start depth stream.", LogSeverity.Error);

        // Set up NiTE
        NiTE.Status result;

        try
        {
            result = NiTE.Initialize();
            BodyTracker = UserTracker.Create(CurrentDevice);

            BodyTracker.OnNewData += BodyTracker_OnNewData;

            InitException = null;
            IsInitialized = true;
        }
        catch (Exception ex)
        {
            result = NiTE.Status.Error;
            Host.Log(ex);

            InitException = ex;
            IsInitialized = false;
        }

        switch (result)
        {
            case NiTE.Status.Ok:
                Host.Log($"Tried to initialize the Kinect sensor with status: {DeviceStatusString}");
                break;
            case NiTE.Status.Error or NiTE.Status.BadUserId:
                Host.Log($"Couldn't initialize the Kinect sensor! Status: {DeviceStatusString}", LogSeverity.Warning);
                break;
            default:
                Host.Log("Tried to initialize the Kinect, but a native exception occurred!", LogSeverity.Error);
                break;
        }

        // Request a refresh of the status UI
        Host?.RefreshStatusInterface();
    }

    private void CurrentSensorOnNewFrame(VideoStream stream)
    {
        if (!stream.IsValid || !stream.IsFrameAvailable() || !IsCameraEnabled) return;

        using var frame = stream.ReadFrame();

        if (!frame.IsValid) return;

        try
        {
            var (buffer, width, height, format) = frame
                .GetFrame(VideoFrameRef.CopyBitmapOptions.Force24BitRgb | VideoFrameRef.CopyBitmapOptions.DepthFillShadow);

            if (buffer is null || buffer.Length == 0)
                return;

            var srcRowBytes = width * 3; // replace with frameBuffer.StrideBytes if available

            var info32 = new SKImageInfo(width, height, SKColorType.Bgra8888, SKAlphaType.Opaque);

            if (CameraImage == null || !CameraImage.Info.Equals(info32))
            {
                CameraImage?.Dispose();
                CameraImage = new SKBitmap(info32, SKBitmapAllocFlags.None);
            }

            unsafe
            {
                fixed (byte* srcBase = buffer)
                {
                    var dstBase = (byte*)CameraImage.GetPixels().ToPointer();
                    var dstRowBytes = CameraImage.RowBytes; // typically width * 4

                    for (var y = 0; y < height; y++)
                    {
                        var src = srcBase + y * srcRowBytes;
                        var dst = dstBase + y * dstRowBytes;

                        for (var x = 0; x < width; x++)
                        {
                            var r = src[3 * x + 0];
                            var g = src[3 * x + 1];
                            var b = src[3 * x + 2];

                            // BGRA order with opaque alpha
                            dst[4 * x + 0] = b;
                            dst[4 * x + 1] = g;
                            dst[4 * x + 2] = r;
                            dst[4 * x + 3] = 0xFF;
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Host.Log(ex);
        }
    }

    public void OnDeviceDisconnected(DeviceInfo device)
    {
        if (device != CurrentDevice.DeviceInfo) return;

        Shutdown();

        // Request a refresh of the status UI
        Host?.RefreshStatusInterface();
    }

    public void Shutdown()
    {
        NiTE.Status result;

        try
        {
            Host.Log("Shutting down...");
            IsInitialized = false;

            if (CurrentSensor is not null)
            {
                CurrentSensor.Stop();
                CurrentSensor.OnNewFrame -= CurrentSensorOnNewFrame;
            }

            DepthStream?.Stop();

            if (BodyTracker is not null)
                BodyTracker.OnNewData += BodyTracker_OnNewData;

            NiTE.Shutdown();
            OpenNI.Shutdown();

            result = NiTE.Status.Ok;
        }
        catch (Exception ex)
        {
            Host.Log(ex);
            result = NiTE.Status.Error;
        }

        switch (result)
        {
            case NiTE.Status.Ok:
                Host.Log($"Tried to shutdown the Kinect sensor with status: {DeviceStatusString}");
                break;
            case NiTE.Status.Error:
                Host.Log($"Kinect sensor is already shut down! Status: {DeviceStatusString}", LogSeverity.Warning);
                break;
            default:
                Host.Log("Tried to shutdown the Kinect sensor, but a native exception occurred!", LogSeverity.Error);
                break;
        }
    }

    public void Update()
    {
    }

    private void BodyTracker_OnNewData(UserTracker userTracker)
    {
        if (!userTracker.IsValid)
            return;

        using var frame = userTracker.ReadFrame();

        foreach (var frameUser in frame.Users)
        {
            // Update tracking data
            if (frameUser.IsNew)
            {
                Host.Log($"New user {frameUser.UserId}");
                userTracker.StartSkeletonTracking(frameUser.UserId);
                IsSkeletonTracked = false;
            }
            else if (frameUser.IsLost)
            {
                Host.Log($"Lost user {frameUser.UserId}");
                IsSkeletonTracked = false;
            }
        }

        var user = frame.Users.FirstOrDefault(x => x.IsVisible);
        if (user is null)
        {
            IsSkeletonTracked = false;
            return;
        }

        if (user.Skeleton.State is not Skeleton.SkeletonState.None)
        {
            var head = user.Skeleton.GetJoint(SkeletonJoint.JointType.Head);
            IsSkeletonTracked = head.IsValid && head.Position.ToVector() != Vector3.Zero;

            if (!IsSkeletonTracked)
                return;

            // Update joints
            foreach (var (x, index) in Enum.GetValues<TrackedJointType>()
                         .Where(x => x is not TrackedJointType.JointManual)
                         .Select((t, i) => (user.Skeleton.GetJoint(t.AsNui()), i)))
            {
                var joint = TrackedJoints[index];
                joint.TrackingState = user.Skeleton.State switch
                {
                    Skeleton.SkeletonState.Tracked when x.PositionConfidence > 0.4f && x.IsValid => TrackedJointState.StateTracked,
                    Skeleton.SkeletonState.Tracked => TrackedJointState.StateInferred,
                    _ => TrackedJointState.StateNotTracked
                };

                joint.Position = x.Position.ToVector().Safe();
                joint.Orientation = x.Orientation.ToQuaternion().Safe();
            }

            UpdateActions(user.Skeleton);
        }
        else
        {
            IsSkeletonTracked = false;
        }
    }

    private void UpdateActions(Skeleton skeleton)
    {
        // Update gestures
        if (skeleton is null)
            return;

        try
        {
            var shoulderLeft = skeleton.GetJoint(SkeletonJoint.JointType.LeftShoulder).Position.ToVector();
            var shoulderRight = skeleton.GetJoint(SkeletonJoint.JointType.RightShoulder).Position.ToVector();
            var elbowLeft = skeleton.GetJoint(SkeletonJoint.JointType.LeftElbow).Position.ToVector();
            var elbowRight = skeleton.GetJoint(SkeletonJoint.JointType.RightElbow).Position.ToVector();
            var handLeft = skeleton.GetJoint(SkeletonJoint.JointType.LeftHand).Position.ToVector();
            var handRight = skeleton.GetJoint(SkeletonJoint.JointType.RightHand).Position.ToVector();

            // >0.9f when elbow is not bent and the arm is straight : LEFT
            var armDotLeft = Vector3.Dot(
                Vector3.Normalize(elbowLeft - shoulderLeft),
                Vector3.Normalize(handLeft - elbowLeft));

            // >0.9f when the arm is pointing down : LEFT
            var armDownDotLeft = Vector3.Dot(
                new Vector3(0.0f, -1.0f, 0.0f),
                Vector3.Normalize(handLeft - elbowLeft));

            // >0.4f <0.6f when the arm is slightly tilted sideways : RIGHT
            var armTiltDotLeft = Vector3.Dot(
                Vector3.Normalize(shoulderLeft - shoulderRight),
                Vector3.Normalize(handLeft - elbowLeft));

            // >0.9f when elbow is not bent and the arm is straight : LEFT
            var armDotRight = Vector3.Dot(
                Vector3.Normalize(elbowRight - shoulderRight),
                Vector3.Normalize(handRight - elbowRight));

            // >0.9f when the arm is pointing down : RIGHT
            var armDownDotRight = Vector3.Dot(
                new Vector3(0.0f, -1.0f, 0.0f),
                Vector3.Normalize(handRight - elbowRight));

            // >0.4f <0.6f when the arm is slightly tilted sideways : RIGHT
            var armTiltDotRight = Vector3.Dot(
                Vector3.Normalize(shoulderRight - shoulderLeft),
                Vector3.Normalize(handRight - elbowRight));

            /* Trigger the detected gestures */
            if (TrackedJoints[(int)TrackedJointType.JointHandLeft].SupportedInputActions.IsUsed(0, out var pauseActionLeft))
                Host.ReceiveKeyInput(pauseActionLeft, _pauseDetectorLeft.Update(armDotLeft > 0.9f && armTiltDotLeft is > 0.4f and < 0.7f));

            if (TrackedJoints[(int)TrackedJointType.JointHandRight].SupportedInputActions.IsUsed(0, out var pauseActionRight))
                Host.ReceiveKeyInput(pauseActionRight, _pauseDetectorRight.Update(armDotRight > 0.9f && armTiltDotRight is > 0.4f and < 0.7f));

            if (TrackedJoints[(int)TrackedJointType.JointHandLeft].SupportedInputActions.IsUsed(1, out var pointActionLeft))
            {
                Host.ReceiveKeyInput(pointActionLeft, _pointDetectorLeft
                    .Update(armDotLeft > 0.9f && armTiltDotLeft is > -0.5f and < 0.5f && armDownDotLeft is > -0.3f and < 0.7f));
            }

            if (TrackedJoints[(int)TrackedJointType.JointHandRight].SupportedInputActions.IsUsed(1, out var pointActionRight))
            {
                Host.ReceiveKeyInput(pointActionRight, _pointDetectorRight
                    .Update(armDotRight > 0.9f && armTiltDotRight is > -0.5f and < 0.5f && armDownDotRight is > -0.3f and < 0.7f));
            }
        }
        catch (Exception ex)
        {
            Host?.Log(ex);
        }
    }

    public void SignalJoint(int jointId)
    {
        // ignored
    }

    public Func<SKBitmap> GetCameraImage
    {
        get => () => CameraImage;
    }

    public Func<bool> GetIsCameraEnabled
    {
        get => () => IsCameraEnabled;
    }

    public Action<bool> SetIsCameraEnabled
    {
        get => value => IsCameraEnabled = value;
    }

    public Func<Vector3, System.Drawing.Size> MapCoordinateDelegate
    {
        get => MapCoordinate;
    }

    private System.Drawing.Size MapCoordinate(Vector3 arg)
    {
        if (!IsInitialized || BodyTracker is null || CurrentSensor is null || DepthStream is null ||
            !CurrentSensor.IsValid || !DepthStream.IsValid) return System.Drawing.Size.Empty;

        // Convert from real-world to depth coordinates
        if (BodyTracker.ConvertJointCoordinatesToDepth(arg.X, arg.Y, arg.Z, out var px, out var py)
            is not NiTE.Status.Ok) return System.Drawing.Size.Empty;

        // Round X/Y to integers, clamp Z to ushort (mm)
        var dx = (int)Math.Round(px);
        var dy = (int)Math.Round(py);

        return new System.Drawing.Size(dx, dy + 30); // Offset a bit...

        //var dz = (ushort)Math.Max(0, Math.Min(ushort.MaxValue, (int)Math.Round(arg.Z)));

        //if (CoordinateConverter.ConvertDepthToColor(DepthStream, CurrentSensor, dx, dy, dz,
        //        out var cx, out var cy) is not OpenNI.Status.Ok) return System.Drawing.Size.Empty;

        //return new System.Drawing.Size(cx, cy);
    }
}

internal static class Utils
{
    public static Quaternion Safe(this Quaternion q)
    {
        return (q.X is 0 && q.Y is 0 && q.Z is 0 && q.W is 0) ||
               float.IsNaN(q.X) || float.IsNaN(q.Y) || float.IsNaN(q.Z) || float.IsNaN(q.W) ?
            Quaternion.Identity // Return a placeholder quaternion
            :
            q; // If everything is fine, return the actual orientation
    }

    public static Vector3 Safe(this Vector3 v)
    {
        return float.IsNaN(v.X) || float.IsNaN(v.Y) || float.IsNaN(v.Z) ?
            Vector3.Zero // Return a placeholder position vector
            :
            v; // If everything is fine, return the actual orientation
    }

    public static T At<T>(this SortedSet<T> set, int at)
    {
        return set.ElementAt(at);
    }

    public static bool At<T>(this SortedSet<T> set, int at, out T result)
    {
        try
        {
            result = set.ElementAt(at);
        }
        catch
        {
            result = default;
            return false;
        }

        return true;
    }

    public static bool IsUsed(this SortedSet<IKeyInputAction> set, int at)
    {
        return set.At(at, out var action) && (Kinect360.HostStatic?.CheckInputActionIsUsed(action) ?? false);
    }

    public static bool IsUsed(this SortedSet<IKeyInputAction> set, int at, out IKeyInputAction action)
    {
        return set.At(at, out action) && (Kinect360.HostStatic?.CheckInputActionIsUsed(action) ?? false);
    }

    public static TrackedJoint WithName(this TrackedJoint joint, string name)
    {
        return new TrackedJoint
        {
            Name = name,
            Role = joint.Role,
            Acceleration = joint.Acceleration,
            AngularAcceleration = joint.AngularAcceleration,
            AngularVelocity = joint.AngularVelocity,
            Orientation = joint.Orientation,
            Position = joint.Position,
            SupportedInputActions = joint.SupportedInputActions,
            TrackingState = joint.TrackingState,
            Velocity = joint.Velocity
        };
    }
}

public static class Extensions
{
    public static Vector3 ToVector(this Point3D point)
    {
        return new Vector3(point.X, point.Y, point.Z) * 0.001f;
    }

    public static Quaternion ToQuaternion(this NiTEWrapper.Quaternion quat)
    {
        return new Quaternion(quat.X, quat.Y, quat.Z, quat.W);
    }

    public static SkeletonJoint.JointType AsNui(this TrackedJointType type)
    {
        return type switch
        {
            TrackedJointType.JointHead => SkeletonJoint.JointType.Head,
            TrackedJointType.JointNeck => SkeletonJoint.JointType.Neck,
            TrackedJointType.JointSpineShoulder => SkeletonJoint.JointType.Neck,
            TrackedJointType.JointShoulderLeft => SkeletonJoint.JointType.LeftShoulder,
            TrackedJointType.JointElbowLeft => SkeletonJoint.JointType.LeftElbow,
            TrackedJointType.JointWristLeft => SkeletonJoint.JointType.LeftHand,
            TrackedJointType.JointHandLeft => SkeletonJoint.JointType.LeftHand,
            TrackedJointType.JointHandTipLeft => SkeletonJoint.JointType.LeftHand,
            TrackedJointType.JointThumbLeft => SkeletonJoint.JointType.LeftHand,
            TrackedJointType.JointShoulderRight => SkeletonJoint.JointType.RightShoulder,
            TrackedJointType.JointElbowRight => SkeletonJoint.JointType.RightElbow,
            TrackedJointType.JointWristRight => SkeletonJoint.JointType.RightHand,
            TrackedJointType.JointHandRight => SkeletonJoint.JointType.RightHand,
            TrackedJointType.JointHandTipRight => SkeletonJoint.JointType.RightHand,
            TrackedJointType.JointThumbRight => SkeletonJoint.JointType.RightHand,
            TrackedJointType.JointSpineMiddle => SkeletonJoint.JointType.Torso,
            TrackedJointType.JointSpineWaist => SkeletonJoint.JointType.Torso,
            TrackedJointType.JointHipLeft => SkeletonJoint.JointType.LeftHip,
            TrackedJointType.JointKneeLeft => SkeletonJoint.JointType.LeftKnee,
            TrackedJointType.JointFootLeft => SkeletonJoint.JointType.LeftFoot,
            TrackedJointType.JointFootTipLeft => SkeletonJoint.JointType.LeftFoot,
            TrackedJointType.JointHipRight => SkeletonJoint.JointType.RightHip,
            TrackedJointType.JointKneeRight => SkeletonJoint.JointType.RightKnee,
            TrackedJointType.JointFootRight => SkeletonJoint.JointType.RightFoot,
            TrackedJointType.JointFootTipRight => SkeletonJoint.JointType.RightFoot,
            _ => SkeletonJoint.JointType.Head
        };
    }
}
