using UnityEngine;
using UnityEditor;
using System.Net;
using System.Net.Sockets;
using System;
using System.Text;
using System.Collections.Generic;

public class vibration_state
{
    public Socket sock;
    public const int BUFFER_SIZE = 15;
    public byte[] buffer = new byte[BUFFER_SIZE];

    private IPEndPoint ep;
    public EndPoint tempEp;

    public vibration_state(string ip, int port)
    {
        sock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, 
            ProtocolType.Udp);
        ep = new IPEndPoint(IPAddress.Any, port);
        sock.Bind(ep);
        Debug.Log("Socket bound");

        tempEp = ep;
    }
}

struct controller_pose
{
    public Vector3 position;
    public Quaternion orientation;
}

class vive_pose
{
    public controller_pose current_pose;
    public controller_pose initial_pose;

    public vive_pose(Transform transform)
    {
        UpdateInitialPose(transform);
    }

    public void UpdateInitialPose(Transform transform)
    {
        initial_pose.position = transform.position;
        initial_pose.orientation = transform.rotation;
        current_pose.position = initial_pose.position;
        current_pose.orientation = initial_pose.orientation;
    }

    private Vector3 UnityToRobotFramePosition(Vector3 p)
    {
        // Josh
        return new Vector3(-p.y, -p.x, p.z);
    }

    private Quaternion UnityToRobotFrameOrientation(Quaternion q)
    {
        // Josh
        Vector3 eulerAngles = q.eulerAngles;
        Vector3 newEuler = new Vector3(eulerAngles.y, eulerAngles.x, -eulerAngles.z);
        Quaternion newQuat = new Quaternion
        {
            eulerAngles = newEuler
        };
        return newQuat;
    }

    public void UpdateCurrentPose(Transform transform)
    {
        current_pose.position = transform.position;
        current_pose.orientation = transform.rotation;

        // Offset position to initial frame
        current_pose.position -= initial_pose.position;
        current_pose.position = Quaternion.Inverse(initial_pose.orientation) *
            current_pose.position;

        current_pose.orientation = QuaternionUtil.DispQ(initial_pose.orientation,
            current_pose.orientation);

        // Convert to robot frame
        current_pose.position = UnityToRobotFramePosition(current_pose.position);
        current_pose.orientation = UnityToRobotFrameOrientation(current_pose.orientation);
    }
}


public class vive_broadcaster : MonoBehaviour
{

    public string IP;

    private Socket sock;
    private IPEndPoint sendEp;

    private string outputString;
    private bool twoControllers;

    private short loopCount = 0;

    private GameObject viveControllerL;
    private GameObject viveControllerR;
    private vive_buttons contrStatesL;
    private vive_buttons contrStatesR;
    private vive_pose poseL;
    private vive_pose poseR;
    private List<vive_buttons> controllers;
    private Transform transformL;
    private Transform transformR;

    void Start()
    {
        sock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        sendEp = new IPEndPoint(IPAddress.Parse(IP), 11000);

        viveControllerL = GameObject.Find("Controller (left)");
        viveControllerR = GameObject.Find("Controller (right)");

        contrStatesL = viveControllerL.GetComponent<vive_buttons>();
        contrStatesR = viveControllerR.GetComponent<vive_buttons>();
        controllers = new List<vive_buttons> { contrStatesL, contrStatesR };

        transformL = viveControllerL.transform;
        transformR = viveControllerR.transform;
        poseL = new vive_pose(transformL);
        poseR = new vive_pose(transformR);

        //VibrationState vibr_state = new VibrationState(IP, 11100);
        //vibr_state.sock.BeginReceiveFrom(vibr_state.buffer, 0, VibrationState.BUFFER_SIZE, SocketFlags.None, 
        //ref vibr_state.temp_ep, new AsyncCallback(handleVibrationData), vibr_state);
        //print("Begin receive data");
    }

    void Update()
    {
        // TODO: Figure out why controller connection is not registering

        bool leftConnected = contrStatesL.poseAction.GetDeviceIsConnected(contrStatesL.handType);
        bool rightConnected = contrStatesR.poseAction.GetDeviceIsConnected(contrStatesR.handType);

        if (!(leftConnected || rightConnected))
        {
            return;
        }

        bool triggerStop = contrStatesL.stop || contrStatesR.stop;
        if (triggerStop)
        {
            Debug.Log("Application exit triggered. Exiting...");
            EditorApplication.isPlaying = false;
            return;
        }

        // TODO: For better handling, set lastConnected variable for 
        // each hand and check whether the device is newly connected.
        // If so, run a loop to get a new initial pose.
        // Currently, turning on the controller while the program is
        // running will not set the initial pose correctly.
        if (loopCount < 5)
        {
            poseL.UpdateInitialPose(transformL);
            poseR.UpdateInitialPose(transformR);

            loopCount++;
            return;
        }

        poseL.UpdateCurrentPose(transformL);
        poseR.UpdateCurrentPose(transformR);

        // Output string formats: 
        // twoControllers?;pos.x,y,z;quat.w,x,y,z;grab?;clutch?;
        // twoContr?;[poseR];[poseL];grab?;clutch?;
        outputString = "";

        twoControllers = leftConnected && rightConnected;
        outputString += String.Format("{0};", Convert.ToUInt16(twoControllers));

        if (rightConnected)
        {
            outputString += PoseOutput(poseR);
        }

        if (leftConnected)
        {
            outputString += PoseOutput(poseL);
        }

        outputString += GrabOutput();
        outputString += ClutchOutput();

        CheckVibration();

        Debug.Log(outputString);

        byte[] sendBuffer = Encoding.ASCII.GetBytes(outputString);
        sock.SendTo(sendBuffer, sendEp);
    }

    private void OnApplicationQuit()
    {
        sock.Close();
    }

    private String PoseOutput(vive_pose pose)
    {
        Vector3 goalPos = pose.current_pose.position;
        Quaternion goalOrient = pose.current_pose.orientation;
        String outStr = String.Format("{0},{1},{2};", goalPos.x, goalPos.y, goalPos.z);
        outStr += String.Format("{0},{1},{2},{3};", goalOrient.w, goalOrient.x, goalOrient.y, goalOrient.z);
        return outStr;
    }

    private String GrabOutput()
    {
        bool grab = contrStatesL.grab || contrStatesR.grab;
        return String.Format("{0};", Convert.ToUInt16(grab));
    }

    private String ClutchOutput()
    {
        bool clutch = contrStatesL.clutch || contrStatesR.clutch;
        return String.Format("{0};", Convert.ToUInt16(clutch));
    }

    private void CheckVibration()
    {
        foreach (vive_buttons contr in controllers)
        {
            bool freqMode = contr.freqMode;
            Vector2 vibrLevel = contr.vibrLevel;

            if (vibrLevel.magnitude == 0)
            {
                continue;
            }

            if (!freqMode)
            {
                contr.TriggerHapticPulse(0.1f, 200, vibrLevel.y);
                print("Amplitude: " + vibrLevel.y);
            }
            else
            {
                vibrLevel *= 320;
                contr.TriggerHapticPulse(0.05f, vibrLevel.y, 0.3f);
                print("Frequency: " + vibrLevel.y);
            }
        }
    }

    void HandleVibrationData(IAsyncResult result)
    {
        print("Entered handleVibration");
        vibration_state vs = (vibration_state)result.AsyncState;

        print("About to start receiving data");
        int dataSize = vs.sock.EndReceiveFrom(result, ref vs.tempEp);
        print(String.Format("{0} bytes of data received", dataSize));

        float vibrValue = (float)Convert.ToDouble(Encoding.UTF8.GetString(vs.buffer, 
            0, vibration_state.BUFFER_SIZE));
        if (vibrValue > 1)
        {
            print("Vibration value too high (> 1)");
        }
        else if (vibrValue > 0)
        {
            print(String.Format("Vibration level: {0}", vibrValue));
            contrStatesL.TriggerHapticPulse(0.1f, 175, vibrValue);
            contrStatesR.TriggerHapticPulse(0.1f, 175, vibrValue);
        }
        else
        {
            Debug.LogError("Vibration value invalid");
        }

        vs.sock.BeginReceiveFrom(vs.buffer, 0, vibration_state.BUFFER_SIZE, SocketFlags.None,
            ref vs.tempEp, new AsyncCallback(HandleVibrationData), vs);
    }
}
