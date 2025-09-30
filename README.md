<h1 dir=auto>
<b>Kinect 360 (NiTE)</b>
<a style="color:#9966cc;" href="https://github.com/KinectToVR/Amethyst">Amethyst</a>
<text>device plugin</text>
</h1>

## **License**
This project is licensed under the GNU GPL v3 License 

## **Overview**
This repo is a pure implementation of the `ITrackingDevice` interface,  
providing Amethyst support for the Xbox One Kinect, using the 2.0 SDK.  
Both the handler and the plugin itself ([available here](https://github.com/KinectToVR/plugin_KinectOne/tree/main/plugin_KinectOne)) are written in C#

## **Downloads**
You're going to find built plugins in [repo Releases](https://github.com/KinectToVR/plugin_KinectOne/releases/latest).

## **Wanna make one too? (K2API Devices Docs)**
[This repository](https://github.com/KinectToVR/Amethyst.Plugins.Templates) contains templates for plugin types supported by Amethyst.<br>
Install the templates by `dotnet new install Amethyst.Plugins.Templates::1.2.0`  
and use them in Visual Studio (recommended) or straight from the DotNet CLI.  
The project templates already contain most of the needed documentation,  
although please feel free to check out [the official wesite](https://docs.k2vr.tech/) for more docs sometime.

The build and publishment workflow is the same as in this repo (excluding vendor deps).  