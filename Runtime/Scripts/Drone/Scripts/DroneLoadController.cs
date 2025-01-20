using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Assertions;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;
using VehicleComponents.Actuators;
using Rope;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Accord.Math.Optimization;
using UnityEngine.SceneManagement;

using System.Linq;
public class DroneLoadController: MonoBehaviour 
{
    [Header("Basics")]
    [Tooltip("Baselink of the drone")]
    public GameObject BaseLink;
    // [Tooltip("Load's connection point to the rope")]
    // public float ControlFrequency = 50f;
    // [Tooltip("The maximum distance error between the load and the target position, kind of controls the aggressiveness of the maneuvers.")]
    // public float DistanceErrorCap = 10f;
    // private Vector<double> startingPosition = null;
    public float MaxVelocityWithTrackingTarget = 1f;
    // public float MaxAccelerationWithTrackingTarget = 1f;
    public float DecelerationDistance = 1f;

    [Header("Tracking")]
    [Tooltip("An object to follow")]
    public Transform TrackingTargetTF;
    
    
    [Header("Load")]
    [Tooltip("The rope object that this drone is expected to get connected, maybe. Will be used to check for attachment state and such.")]
    public Transform Rope; // TODO remove this requirement.
    [Tooltip("The position of where the load is attached to the rope. rope_link on SAM")]
    public Transform LoadLinkTF; // The position of the AUV is taken at the base of the rope

    [Header("Props")]
    public Transform PropFR, PropFL, PropBR, PropBL;
    
    [Header("Trajectories")]
    public bool AttackTheBuoy = false;

    public bool ContinueTrajectory = false; // ContinueMinimumSnapTrajectory
    public bool Figure8 = false;
    public bool LogTrajectory = false;
    public bool Helix = false;
    public bool RepeatTest = false;
    private double TrajectoryStartTime = 0;
    private double CatchStartTime = 0; // for CatchStartTime start time
    

	Propeller[] propellers;
    float[] propellers_rpms;
    ArticulationBody base_link_ab;
    ArticulationBody load_link_ab;
    Matrix<double> R_sb_d_prev;
    Matrix<double> R_sb_c_prev;
    Vector<double> W_b_d_prev;
    Vector<double> W_b_c_prev;
    Vector<double> q_c_prev;
    Vector<double> q_c_dot_prev;
    int times1 = 0;
    int times2 = 0;

    int rope_insideWindCount = 0;

    ////////////////// SYSTEM SPECIFIC //////////////////
    // Quadrotor parameters
    double mQ;
    double d;
    Matrix<double> J;
    float c_tau_f;
    Matrix<double> T;
    Matrix<double> T_inv;
    Matrix<double> Q;
    const int NUM_PROPS = 4;

    // Load parameters
    double mL;
    double l;

    // Gains
    double kx;
    double kv;
    double kR;
    double kW;
    double kq;
    double kw;
    /////////////////////////////////////////////////////

    // Simulation parameters
    double g;
    Vector<double> e3;
    float dt;

    // Min snap trajectory
    double[] coeffsX = new double[6];
    double[] coeffsY = new double[6];
    double[] coeffsZ = new double[6];
    int min_snap_flag=0;
    double aiming_time = 5;
    double catching_time = 3; 
    double forward_time = 0.5; 
    double lifting_time = 3; 

    double total_MST_time; 
    
    Vector<double> x_s_d_last; // waiting amd aiming

    double Tp = 0; // progress_time

    // Continuous Min snap trajectory
    private List<double[]> trajectoryCoefficients_x; // To store coefficients for each trajectory segment
    private List<double[]> trajectoryCoefficients_y; // To store coefficients for each trajectory segment
    private List<double[]> trajectoryCoefficients_z; // To store coefficients for each trajectory segment
    private List<double> MST_time_stamp; // To store time stamps for each waypoint
    
    //int spaceBarPressCount = 0;
    
    // Downward air flow
    // public float smallRadius = 0.5f;
    // public float largeRadius = 0.5f;
    // public float height = 0.5f;

    private LineRenderer lineRenderer;
    public Vector3[] Gizmos_points;


    // Logging
    string filePath = Application.dataPath + "/../../SMARCUnityAssets/Logs/log.csv";
    TextWriter tw;

	// Use this for initialization
	void Start() 
    {
		propellers = new Propeller[4];
		propellers[0] = PropFL.GetComponent<Propeller>();
		propellers[1] = PropFR.GetComponent<Propeller>();
        propellers[2] = PropBR.GetComponent<Propeller>();
        propellers[3] = PropBL.GetComponent<Propeller>();

        base_link_ab = BaseLink.GetComponent<ArticulationBody>();

        R_sb_d_prev = DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        R_sb_c_prev = DenseMatrix.OfArray(new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        W_b_d_prev = DenseVector.OfArray(new double[] { 0, 0, 0 });
        W_b_c_prev = DenseVector.OfArray(new double[] { 0, 0, 0 });
        q_c_prev = DenseVector.OfArray(new double[] { 0, 0, 0 });
        q_c_dot_prev = DenseVector.OfArray(new double[] { 0, 0, 0 });

		propellers_rpms = new float[] { 0, 0, 0, 0 };

        if(LoadLinkTF != null) load_link_ab = LoadLinkTF.GetComponent<ArticulationBody>();
        
        ////////////////// SYSTEM SPECIFIC //////////////////
        // Quadrotor parameters
        mQ = base_link_ab.mass; // Quadrotor mass (kg)
        d = 0.315; // Distance from the center of the quadrotor to each propeller (assumes square prop configuration) (m)
        J = DenseMatrix.OfArray(new double[,] { { base_link_ab.inertiaTensor.x, 0, 0 }, { 0, base_link_ab.inertiaTensor.z, 0 }, { 0, 0, base_link_ab.inertiaTensor.y } }); // Inertia tensor of the quadrotor in ENU (kg m^2)
        c_tau_f = 8.004e-2f; // Torque to force ratio of the propellers (also found in Propeller.cs, TODO: make this one variable) (m)

        T = DenseMatrix.OfArray(new double[,] { { 1, 1, 1, 1 },
                                                    { d, 0, -d, 0 },
                                                    { 0, -d, 0, d },
                                                    { c_tau_f, -c_tau_f, c_tau_f, -c_tau_f } }); // Mapping from propeller forces to the equivalent wrench (similar version found in the paper)
        T_inv = T.Inverse();
        Q = T.Transpose()*T;

        // Load parameters
        mL = 0; // Load mass (sum of all mass elements on sam) ~15 kg
        if(LoadLinkTF != null)
        {
            ArticulationBody[] sam_ab_list = LoadLinkTF.root.gameObject.GetComponentsInChildren<ArticulationBody>();
            foreach (ArticulationBody sam_ab in sam_ab_list) 
            {
                if (sam_ab.enabled) {
                    mL += sam_ab.mass;
                }
            }
        }

        // Plot downward airflow
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.positionCount = 0;



        // Rope length l is calculated dynamically
        /////////////////////////////////////////////////////

        // Simulation parameters
        g = Physics.gravity.magnitude;
        e3 = DenseVector.OfArray(new double[] { 0, 0, 1 });
        dt = Time.fixedDeltaTime;//1f/ControlFrequency; 

        // Log the time between FixedUpdate calls
        //Debug.Log($"FixedUpdate interval (seconds): {Time.fixedDeltaTime}");   = 0.02 sec
        
        tw = new StreamWriter(filePath, false);
        tw.WriteLine("t,x_s1,x_s2,x_s3,x_s_d1,x_s_d2,x_s_d3,propellers_rpms1,propellers_rpms2,propellers_rpms3,propellers_rpms4,rollRad,pitchRad,yawRad,v_s1,v_s2,v_s3,v_s_d1,v_s_d2,v_s_d3,insideCount");
        tw.Close();

	}
	
	// Update is called once per frame
	void FixedUpdate() 
    {
        //float startTime = Time.realtimeSinceStartup;
		ComputeRPMs();
        //float endTime = Time.realtimeSinceStartup;
        //Debug.Log($"Execution Time: {(endTime - startTime) * 1000} ms");
        ApplyRPMs();

        // Check if space bar is pressed
        // if (Input.GetKeyDown(KeyCode.Space))
        // {
        //     spaceBarPressCount++;
        //     Debug.Log($"Space Bar Pressed: {spaceBarPressCount} times");
        //     // Check if the space bar press threshold is met
        //     if (spaceBarPressCount >= 3)
        //     {
        //         ResetScene();
        //     }
        // }
	}

    private void ResetScene()
    {
        // Reset the scene and variables
        Debug.Log("Resetting Scene...");
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        // Reset the counter and space bar press count
        //spaceBarPressCount = 0;
        //timer = 0f;
    }

    (double, Vector<double>) SuspendedLoadControl()
    {
        double f;
        Vector<double> M;

        ////////////////// SYSTEM SPECIFIC //////////////////
        // Gains
        kx = 16*mQ;
        kv = 5.6*mQ;
        kR = 8.81;
        kW = 0.5;
        kq = 2;
        kw = 0.5;
        /////////////////////////////////////////////////////
        
        // Quadrotor states
        Vector<double> xQ_s = BaseLink.transform.position.To<ENU>().ToDense();
        Vector<double> vQ_s = base_link_ab.velocity.To<ENU>().ToDense();
        Matrix<double> R_sb = DenseMatrix.OfArray(new double[,] { { BaseLink.transform.right.x, BaseLink.transform.forward.x, BaseLink.transform.up.x },
                                                                { BaseLink.transform.right.z, BaseLink.transform.forward.z, BaseLink.transform.up.z },
                                                                { BaseLink.transform.right.y, BaseLink.transform.forward.y, BaseLink.transform.up.y } });
        Vector<double> W_b = -1f*(BaseLink.transform.InverseTransformDirection(base_link_ab.angularVelocity)).To<ENU>().ToDense();

        // Load states
        Vector<double> xL_s = LoadLinkTF.position.To<ENU>().ToDense();
        Vector<double> vL_s = load_link_ab.velocity.To<ENU>().ToDense();
        l = (xL_s - xQ_s).Norm(2); // TODO: Figure out the fixed rope length from the rope object, the controller should work even with stretching
        Vector<double> q = (xL_s - xQ_s)/l;
        Vector<double> q_dot = (vL_s - vQ_s)/l;

        // Desired states
        Vector<double> xL_s_d;
        Vector<double> vL_s_d;
        Vector<double> aL_s_d;
        Vector<double> b1d_d;
        (xL_s_d, vL_s_d, aL_s_d) = TrackingTargetTrajectory(TrackingTargetTF.position.To<ENU>().ToDense(), xL_s, vL_s);
        
        // Figure 8
        if (Figure8) {
            (xL_s_d, vL_s_d, aL_s_d) = Figure8Trajectory(Time.time - TrajectoryStartTime);
        }

        Vector<double> b1d = DenseVector.OfArray(new double[] { Math.Sqrt(2)/2, -Math.Sqrt(2)/2, 0 });
        
        // Helix
        if (Helix) {
            (xL_s_d, vL_s_d, aL_s_d, b1d_d) = HelixTrajectory(Time.time - TrajectoryStartTime);
            b1d = b1d_d;
        }

        if (!Figure8 && !Helix) {
            TrajectoryStartTime = Time.time;
        }
        
        // Logging
        if (LogTrajectory) {
            tw = new StreamWriter(filePath, true);
            tw.WriteLine($"{Time.time},{xL_s[0]},{xL_s[1]},{xL_s[2]},{xL_s_d[0]},{xL_s_d[1]},{xL_s_d[2]}");
            tw.Close();
        }

        // Load position controller
        Vector<double> ex = xL_s - xL_s_d;
        Vector<double> ev = vL_s - vL_s_d;

        Vector<double> A = -kx*ex - kv*ev + (mQ+mL)*(aL_s_d + g*e3) + mQ*l*(q_dot*q_dot)*q;
        Vector<double> q_c = -A/A.Norm(2);
        Vector<double> q_c_dot = DenseVector.OfArray(new double[] { 0, 0, 0 });//(q_c - q_c_prev)/dt;
        Vector<double> q_c_ddot = DenseVector.OfArray(new double[] { 0, 0, 0 });//(q_c_dot - q_c_dot_prev)/dt;
        Vector<double> F_n = (A*q)*q;
        Debug.DrawRay(ToUnity(xQ_s), ToUnity(q_c), Color.magenta);

        // Load attitude controller
        Vector<double> eq = _Hat(q)*_Hat(q)*q_c;
        Vector<double> eq_dot = q_dot - _Cross(_Cross(q_c, q_c_dot), q);
        
        Vector<double> F_pd = -kq*eq - kw*eq_dot;
        Vector<double> F_ff = mQ*l*(q*_Cross(q_c, q_c_dot))*_Cross(q, q_dot) + mQ*l*_Cross(_Cross(q_c, q_c_ddot), q);
        Vector<double> F_for_f = F_n - F_pd - F_ff;
        
        F_n = -(q_c*q)*q;
        Vector<double> F_for_M = F_n - F_pd - F_ff;
        
        // Quadrotor attitude controller
        Vector<double> b3c = F_for_M/F_for_M.Norm(2);
        Vector<double> b1c = -_Cross(b3c, _Cross(b3c, b1d))/_Cross(b3c, b1d).Norm(2);
        Vector<double> b2c = _Cross(b3c, b1c);
        Matrix<double> R_sb_c = DenseMatrix.OfArray(new double[,] { { b1c[0], b2c[0], b3c[0] },
                                                                    { b1c[1], b2c[1], b3c[1] },
                                                                    { b1c[2], b2c[2], b3c[2] } });

        Vector<double> W_b_c = _Vee(_Logm3(R_sb_c_prev.Transpose()*R_sb_c)/dt);
        Vector<double> W_b_c_dot = (W_b_c - W_b_c_prev)/dt;

        Vector<double> eR = 0.5*_Vee(R_sb_c.Transpose()*R_sb - R_sb.Transpose()*R_sb_c);
        Vector<double> eW = W_b - R_sb.Transpose()*R_sb_c*W_b_c;

        f = F_for_f*(R_sb*e3);
        M = -kR*eR - kW*eW + _Cross(W_b, J*W_b) - J*(_Hat(W_b)*R_sb.Transpose()*R_sb_c*W_b_c - R_sb.Transpose()*R_sb_c*W_b_c_dot);
        
        // Save previous values
        R_sb_c_prev = R_sb_c;
        W_b_c_prev = W_b_c;
        q_c_prev = q_c;
        q_c_dot_prev = q_c_dot;

        if (times1 < 2 || M.Norm(2) > 100) // If previous values have not been initialized yet or moments are excessive
        {
            times1++;
            f = 0;
            M = DenseVector.OfArray(new double[] { 0, 0, 0 });
        }

        return (f, M);
    }

    (double, Vector<double>) TrackingControl()
    {
        double f;
        Vector<double> M;

        ////////////////// SYSTEM SPECIFIC //////////////////
        // Gains
        kx = 16*mQ;
        kv = 5.6*mQ;
        kR = 8.81;
        kW = 2.54;
        /////////////////////////////////////////////////////
        
        // Quadrotor states
        Vector<double> x_s = BaseLink.transform.position.To<ENU>().ToDense();
        Vector<double> v_s = base_link_ab.velocity.To<ENU>().ToDense();
        Matrix<double> R_sb = DenseMatrix.OfArray(new double[,] { { BaseLink.transform.right.x, BaseLink.transform.forward.x, BaseLink.transform.up.x },
                                                                { BaseLink.transform.right.z, BaseLink.transform.forward.z, BaseLink.transform.up.z },
                                                                { BaseLink.transform.right.y, BaseLink.transform.forward.y, BaseLink.transform.up.y } });
        Vector<double> W_b = -1f*BaseLink.transform.InverseTransformDirection(base_link_ab.angularVelocity).To<ENU>().ToDense();

        // Desired states
        Vector<double> x_s_d;
        Vector<double> v_s_d;
        Vector<double> a_s_d;
        Vector<double> b1d_d;
        (x_s_d, v_s_d, a_s_d) = TrackingTargetTrajectory(TrackingTargetTF.position.To<ENU>().ToDense(), x_s, v_s);

        // Quadrotor roll pitch yaw
        Quaternion orientation = BaseLink.transform.rotation;
        Vector3 eulerENU = orientation.eulerAngles;

        double roll = eulerENU.x;
        double pitch = eulerENU.y;
        double yaw = eulerENU.z;

        double rollRad = roll;
        double pitchRad = pitch;
        double yawRad = yaw;

        // Rad
        // double rollRad = Mathf.Deg2Rad * roll;
        // double pitchRad = Mathf.Deg2Rad * pitch;
        // double yawRad = Mathf.Deg2Rad * yaw;

        // Figure 8
        if (Figure8) {
            (x_s_d, v_s_d, a_s_d) = Figure8Trajectory(Time.time - TrajectoryStartTime);
        }

        // Helix
        if (Helix) {
            (x_s_d, v_s_d, a_s_d, b1d_d) = HelixTrajectory(Time.time - TrajectoryStartTime);
        }

        if (!Figure8 && !Helix) {
            TrajectoryStartTime = Time.time;
        }

        // Repeat Test to record average result
        if (RepeatTest){
            Debug.Log("Repeat Test...");
            if(AttackTheBuoy == false)
            {
                AttackTheBuoy = true; // start catching    
            }
        }

        // Minum snap trajectory parameters 
        // double T = 10.0; // Total time for trajectory

        if(ContinueTrajectory && Rope != null)
        {
            if(min_snap_flag == 0)
            {
                MST_time_stamp = new List<double> { 0, 5, 10, 11, 15}; // time stamp
                total_MST_time = MST_time_stamp[MST_time_stamp.Count - 1];

                // UAV hovers and aims at the middle of the rope, the rope (Rope.childCount) include 22 segements, 0~21
                Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(20).position).To<ENU>();
                Vector3 p_aim = new Vector3(endPosENU.x-3, endPosENU.y, endPosENU.z+3);  // 1st aiming
                Vector3 p_catch = new Vector3(endPosENU.x - 0.05f, endPosENU.y, endPosENU.z);   // 2nd catch 
                Vector3 p_forward = new Vector3(endPosENU.x + 1.0f, endPosENU.y, endPosENU.z - 0.3f); // 3rd move forward
                Vector3 p_lift = new Vector3(endPosENU.x + 1.5f, endPosENU.y, endPosENU.z+3); // 4th lift
                
                //{1st waypoints, 2nd, 3rd, 4th}    
                var positionsX = new List<double> { (float)x_s[0], (float)p_aim[0], (float)p_catch[0], (float)p_forward[0], (float)p_lift[0]};
                var velocitiesX = new List<double> { 0, 0, 0, 0, 0 };
                var accelerationsX = new List<double> { 0, 0, 0, 0, 0 };
 
                var positionsY = new List<double> { (float)x_s[1], (float)p_aim[1], (float)p_catch[1], (float)p_forward[1], (float)p_lift[1]};
                var velocitiesY = new List<double> { 0, 0, 0, 0, 0 };
                var accelerationsY = new List<double> { 0, 0, 0, 0, 0 };

                var positionsZ = new List<double> { (float)x_s[2], (float)p_aim[2], (float)p_catch[2], (float)p_forward[2], (float)p_lift[2]};
                var velocitiesZ = new List<double> { 0, 0, 0, 0, 0 };
                var accelerationsZ = new List<double> { 0, 0, 0, 0, 0 };     


                // Hard  constraint : UAV will fly exactly at the desired velocity and acceleration 
                //trajectoryCoefficients_x = ContinueMinimumSnapTrajectory.GenerateTrajectory(positionsX, velocitiesX, accelerationsX, MST_time_stamp);
                //trajectoryCoefficients_y = ContinueMinimumSnapTrajectory.GenerateTrajectory(positionsY, velocitiesY, accelerationsY, MST_time_stamp);
                //trajectoryCoefficients_z = ContinueMinimumSnapTrajectory.GenerateTrajectory(positionsZ, velocitiesZ, accelerationsZ, MST_time_stamp);
                
                // Soft constraint
                double lambda = 1e-1; // Adjust this to control how strictly soft constraints are enforced
                trajectoryCoefficients_x = ContinueMinimumSnapTrajectory.GenerateTrajectorySoft(positionsX, velocitiesX, accelerationsX, MST_time_stamp, lambda);
                trajectoryCoefficients_y = ContinueMinimumSnapTrajectory.GenerateTrajectorySoft(positionsY, velocitiesY, accelerationsY, MST_time_stamp, lambda);
                trajectoryCoefficients_z = ContinueMinimumSnapTrajectory.GenerateTrajectorySoft(positionsZ, velocitiesZ, accelerationsZ, MST_time_stamp, lambda);
                //Debug.Log($"trajectoryCoefficients_x: {trajectoryCoefficients_x.Count}"); //4


                // Print all coefficients
                for (int i = 0; i < trajectoryCoefficients_x.Count; i++)
                { 
                    //Debug.Log($"Console Writeline:{i}");
                    //Console.WriteLine($"Segment {i + 1}: {string.Join(", ", trajectoryCoefficients_x[i])}"); 
                    Debug.Log($"Segment {i + 1}: {string.Join(", ", trajectoryCoefficients_x[i])}");

                }
                // double[] trajec_x0 = trajectoryCoefficients_x[0];
                // double[] trajec_x1 = trajectoryCoefficients_x[1];
                // double[] trajec_x2 = trajectoryCoefficients_x[2];
                // double[] trajec_x3 = trajectoryCoefficients_x[3];
            
                // foreach (double value in trajec_x0)
                // {
                //     Debug.Log(value);
                // }
                //Debug.Log($"trajectoryCoefficients_x: {trajec_x0},{trajec_x1},{trajec_x2},{trajec_x3}");
                //Debug.Log(string.Join(", ", trajec_x0,", ", trajec_x1,", ", trajec_x2,", ", trajec_x3));

                min_snap_flag = 1;
                CatchStartTime = Time.time; // reset time
                Debug.Log($"UAV completed trajectory calculation");
            }
            if(min_snap_flag == 1)
            {
                Tp = Time.time - CatchStartTime;

                if(Tp < total_MST_time)
                {
                    var (posX, velX, accX) = ContinueMinimumSnapTrajectory.EvaluateTrajectory(trajectoryCoefficients_x, MST_time_stamp, Tp);
                    var (posY, velY, accY) = ContinueMinimumSnapTrajectory.EvaluateTrajectory(trajectoryCoefficients_y, MST_time_stamp, Tp);
                    var (posZ, velZ, accZ) = ContinueMinimumSnapTrajectory.EvaluateTrajectory(trajectoryCoefficients_z, MST_time_stamp, Tp);
                    x_s_d = DenseVector.OfArray(new double[] { posX, posY, posZ });
                    v_s_d = DenseVector.OfArray(new double[] { velX, velY, velZ });
                    a_s_d = DenseVector.OfArray(new double[] { accX, accY, accZ });
                    x_s_d_last = x_s_d;
                    //Debug.Log($"UAV is catching.....................{Tp} / {total_MST_time}"); 
                    Debug.Log($"posX:{posX:F2},velX:{velX:F2},accX:{accX:F2},Tp:{Tp:F2}");
                }else{
                    x_s_d = x_s_d_last;
                    v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
                    a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });

                    Debug.Log($"UAV complete catching, stay in the last point");
                }
            }
        }



        if(AttackTheBuoy && Rope != null)
        {   // aiming 
            if(min_snap_flag == 0)  
            {
                double T = aiming_time;
                // UAV start position 
                Vector3 startPos = new Vector3((float)x_s[0], (float)x_s[1], (float)x_s[2]);
                Vector3 startVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 startAcc = new Vector3(0.0f, 0.0f, 0.0f);

                // UAV hovers and aims at the middle of the rope, the rope (Rope.childCount) include 22 segements, 0~21
                Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(20).position).To<ENU>();
                Vector3 endPos = new Vector3(endPosENU.x-3, endPosENU.y, endPosENU.z+3);
                Vector3 endVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 endAcc = new Vector3(0.0f, 0.0f, 0.0f);               

                // Calculate minimum snap trajectory coefficients for each axis (x, y, z)
                coeffsX = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.x, startVel.x, startAcc.x, endPos.x, endVel.x, endAcc.x, T);
                coeffsY = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.y, startVel.y, startAcc.y, endPos.y, endVel.y, endAcc.y, T);
                coeffsZ = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.z, startVel.z, startAcc.z, endPos.z, endVel.z, endAcc.z, T);

                min_snap_flag = 1;
                CatchStartTime = Time.time; // reset time
            }
            if (min_snap_flag == 1)
            {
                //aiming_time = aiming_time - 0.1; // move and aim until 5 sec

                Tp = Time.time - CatchStartTime;

                if (Tp < aiming_time)
                {
                //if (aiming_time > 0)
                //{   //Tp = Tp + 0.1;
                    double posX = MinimumSnapTrajectory.EvaluatePolynomial(coeffsX, Tp);
                    double posY = MinimumSnapTrajectory.EvaluatePolynomial(coeffsY, Tp);
                    double posZ = MinimumSnapTrajectory.EvaluatePolynomial(coeffsZ, Tp);

                    // Evaluate velocity (first derivative)
                    double velX = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsX, Tp);
                    double velY = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsY, Tp);
                    double velZ = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsZ, Tp);

                    // Evaluate acceleration (second derivative)
                    double accX = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsX, Tp);
                    double accY = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsY, Tp);
                    double accZ = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsZ, Tp);

                    x_s_d = DenseVector.OfArray(new double[] { posX, posY, posZ });
                    v_s_d = DenseVector.OfArray(new double[] { velX, velY, velZ });
                    a_s_d = DenseVector.OfArray(new double[] { accX, accY, accZ });

                    x_s_d_last = x_s_d;
                    
                //}else if(aiming_time > -15.0f)
                }else if(Tp < aiming_time + 5.0f)
                {
                    x_s_d = x_s_d_last;   // waiting in the last position
                    v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
                    a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
                    //Debug.Log($"UAV is aiming.....................{aiming_time}"); 
                    Debug.Log($"UAV is aiming.....................{Tp} / {aiming_time + 5.0f}");
                }else
                {
                    Debug.Log("Transfer to flag 2"); 
                    min_snap_flag = 2; // complete aiming and move to catching
                    Tp = 0; // Reset progress time to 0 for next motion: cathcing  
                }
            }

            // catching 
            if(min_snap_flag == 2)
            {
                double T = catching_time;
                Vector3 startPos = new Vector3((float)x_s[0], (float)x_s[1], (float)x_s[2]); // current position
                Vector3 startVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 startAcc = new Vector3(0.0f, 0.0f, 0.0f);

                // For now we want the end position to be the middle of the rope,
                // but later it will be refined based on the state estimation and
                // optimal touchdown point

                //Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(Rope.childCount-1).position).To<ENU>();
                Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(20).position).To<ENU>();  // rope position
                Vector3 endPos = new Vector3(endPosENU.x, endPosENU.y, endPosENU.z);
                Vector3 endVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 endAcc = new Vector3(0.0f, 0.0f, 0.0f);
              
                // Calculate minimum snap trajectory coefficients for each axis (x, y, z)
                coeffsX = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.x, startVel.x, startAcc.x, endPos.x, endVel.x, endAcc.x, T);
                coeffsY = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.y, startVel.y, startAcc.y, endPos.y, endVel.y, endAcc.y, T);
                coeffsZ = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.z, startVel.z, startAcc.z, endPos.z, endVel.z, endAcc.z, T);
                min_snap_flag = 3;
                CatchStartTime = Time.time; // reset time
            }
            if(min_snap_flag == 3)  
            {
                Tp = Time.time - CatchStartTime;
                Debug.Log($"UAV is catching.....................{Tp} / {catching_time}"); 
                //catching_time = catching_time - 0.1;
                
                if ( Tp < catching_time)
                {
                //if (catching_time > 0)
                //{    
                    //Tp = Tp + 0.1;
                    //Tp = Time.time - CatchStartTime;
                    double posX = MinimumSnapTrajectory.EvaluatePolynomial(coeffsX, Tp);
                    double posY = MinimumSnapTrajectory.EvaluatePolynomial(coeffsY, Tp);
                    double posZ = MinimumSnapTrajectory.EvaluatePolynomial(coeffsZ, Tp);

                    // Evaluate velocity (first derivative)
                    double velX = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsX, Tp);
                    double velY = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsY, Tp);
                    double velZ = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsZ, Tp);

                    // Evaluate acceleration (second derivative)
                    double accX = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsX, Tp);
                    double accY = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsY, Tp);
                    double accZ = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsZ, Tp);

                    x_s_d = DenseVector.OfArray(new double[] { posX, posY, posZ });
                    v_s_d = DenseVector.OfArray(new double[] { velX, velY, velZ });
                    a_s_d = DenseVector.OfArray(new double[] { accX, accY, accZ });

                    //Debug.Log("UAV is catching..................... catching...................."); 
                    
                }else{
                    min_snap_flag = 4; // complete catching and move to lifting
                    Tp = 0; // Reset progress time to 0 for next motion: lifting  
                }
                //   
            }

            // forward 
            if(min_snap_flag == 4)
            {
                double T = forward_time;
                Vector3 startPos = new Vector3((float)x_s[0], (float)x_s[1], (float)x_s[2]); // current position
                Vector3 startVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 startAcc = new Vector3(0.0f, 0.0f, 0.0f);

                // For now we want the end position to be the middle of the rope,
                // but later it will be refined based on the state estimation and
                // optimal touchdown point

                //Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(Rope.childCount-1).position).To<ENU>();
                Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(20).position).To<ENU>();  // rope position
                Vector3 endPos = new Vector3(endPosENU.x + 1.0f, endPosENU.y, endPosENU.z - 0.3f);
                Vector3 endVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 endAcc = new Vector3(0.0f, 0.0f, 0.0f);
              
                // Calculate minimum snap trajectory coefficients for each axis (x, y, z)
                coeffsX = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.x, startVel.x, startAcc.x, endPos.x, endVel.x, endAcc.x, T);
                coeffsY = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.y, startVel.y, startAcc.y, endPos.y, endVel.y, endAcc.y, T);
                coeffsZ = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.z, startVel.z, startAcc.z, endPos.z, endVel.z, endAcc.z, T);
                min_snap_flag = 5;
                CatchStartTime = Time.time; // reset time
            }
            if(min_snap_flag == 5)  
            {
                Tp = Time.time - CatchStartTime;
                Debug.Log($"UAV is forward.....................{Tp} / {forward_time}"); 
                //catching_time = catching_time - 0.1;
                
                if ( Tp < forward_time)
                {
                //if (catching_time > 0)
                //{    
                    //Tp = Tp + 0.1;
                    //Tp = Time.time - CatchStartTime;
                    double posX = MinimumSnapTrajectory.EvaluatePolynomial(coeffsX, Tp);
                    double posY = MinimumSnapTrajectory.EvaluatePolynomial(coeffsY, Tp);
                    double posZ = MinimumSnapTrajectory.EvaluatePolynomial(coeffsZ, Tp);

                    // Evaluate velocity (first derivative)
                    double velX = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsX, Tp);
                    double velY = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsY, Tp);
                    double velZ = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsZ, Tp);

                    // Evaluate acceleration (second derivative)
                    double accX = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsX, Tp);
                    double accY = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsY, Tp);
                    double accZ = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsZ, Tp);

                    x_s_d = DenseVector.OfArray(new double[] { posX, posY, posZ });
                    v_s_d = DenseVector.OfArray(new double[] { velX, velY, velZ });
                    a_s_d = DenseVector.OfArray(new double[] { accX, accY, accZ });

                    //Debug.Log("UAV is catching..................... forwarding ...................."); 
                    
                }else{
                    min_snap_flag = 6; // complete catching and move to lifting
                    Tp = 0; // Reset progress time to 0 for next motion: lifting  
                }
                //   
            }           
        

            // lifting 
            if(min_snap_flag == 6)
            {
                double T = lifting_time;
                Vector3 startPos = new Vector3((float)x_s[0], (float)x_s[1], (float)x_s[2]); // current position
                Vector3 startVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 startAcc = new Vector3(0.0f, 0.0f, 0.0f);

                // For now we want the end position to be the middle of the rope,
                // but later it will be refined based on the state estimation and
                // optimal touchdown point

                //Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(Rope.childCount-1).position).To<ENU>();
                //Vector3<ENU> endPosENU = 0.5f*(LoadLinkTF.position + Rope.GetChild(20).position).To<ENU>();  // rope position
                Vector3 endPos = new Vector3((float)x_s[0]+1.0f, (float)x_s[1], (float)x_s[2]+1.0f);
                Vector3 endVel = new Vector3(0.0f, 0.0f, 0.0f);
                Vector3 endAcc = new Vector3(0.0f, 0.0f, 0.0f);
              
                // Calculate minimum snap trajectory coefficients for each axis (x, y, z)
                coeffsX = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.x, startVel.x, startAcc.x, endPos.x, endVel.x, endAcc.x, T);
                coeffsY = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.y, startVel.y, startAcc.y, endPos.y, endVel.y, endAcc.y, T);
                coeffsZ = MinimumSnapTrajectory.MinimumSnapCoefficients(startPos.z, startVel.z, startAcc.z, endPos.z, endVel.z, endAcc.z, T);
                min_snap_flag = 7;
                CatchStartTime = Time.time; // reset time
            }
            if(min_snap_flag == 7)  
            {
                Debug.Log($"UAV is lifting .....................{Tp} / {lifting_time}"); 
                //catching_time = catching_time - 0.1;
                Tp = Time.time - CatchStartTime;
                if ( Tp < lifting_time)
                {
                //if (catching_time > 0)
                //{    
                    //Tp = Tp + 0.1;
                    //Tp = Time.time - CatchStartTime;
                    double posX = MinimumSnapTrajectory.EvaluatePolynomial(coeffsX, Tp);
                    double posY = MinimumSnapTrajectory.EvaluatePolynomial(coeffsY, Tp);
                    double posZ = MinimumSnapTrajectory.EvaluatePolynomial(coeffsZ, Tp);

                    // Evaluate velocity (first derivative)
                    double velX = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsX, Tp);
                    double velY = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsY, Tp);
                    double velZ = MinimumSnapTrajectory.EvaluatePolynomialDerivative(coeffsZ, Tp);

                    // Evaluate acceleration (second derivative)
                    double accX = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsX, Tp);
                    double accY = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsY, Tp);
                    double accZ = MinimumSnapTrajectory.EvaluatePolynomialSecondDerivative(coeffsZ, Tp);

                    x_s_d = DenseVector.OfArray(new double[] { posX, posY, posZ });
                    v_s_d = DenseVector.OfArray(new double[] { velX, velY, velZ });
                    a_s_d = DenseVector.OfArray(new double[] { accX, accY, accZ });
                    
                }else{
                    min_snap_flag = 8; // complete catching and move to lifting
                    //Tp = 0; // Reset progress time to 0 for next motion: lifting  
                    LogTrajectory = false;
                    AttackTheBuoy = false;
                    Debug.Log($"switch to suspended control");
                    //Debug.Log($"Reset the scene");
                    //ResetScene();
                }
                //   
            }       

        
            /*
            Vector<double> buoy_w = R_ws*Rope.GetChild(Rope.childCount-1).position.To<NED>().ToDense();
            x_s_d = R_sw*DenseVector.OfArray(new double[] { buoy_w[0], buoy_w[1], Math.Pow(t-4, 2)/16 + buoy_w[2] + 0.16 });
            v_s_d = R_sw*DenseVector.OfArray(new double[] { 0, 0, (t-4)/8 });
            a_s_d = R_sw*DenseVector.OfArray(new double[] { 0, 0, 1/8 });
            */
            //Debug.Log($"x_s_d: {x_s_d[0]:F2},{x_s_d[1]:F2},{x_s_d[2]:F2}"); // desired position
            // Debug.Log($"x_s: {x_s[0]:F2},{x_s[1]:F2},{x_s[2]:F2}"); // desired position
        }
        // Debug.Log($"t: {t}"); // Time

        // Logging
        if (LogTrajectory) {
            tw = new StreamWriter(filePath, true);
            //tw.WriteLine($"{Time.time},{x_s[0]},{x_s[1]},{x_s[2]},{x_s_d[0]},{x_s_d[1]},{x_s_d[2]}");
            if(ContinueTrajectory == true)
            {
                tw.WriteLine($"{Tp},{x_s[0]},{x_s[1]},{x_s[2]},{x_s_d[0]},{x_s_d[1]},{x_s_d[2]},{propellers_rpms[0]},{propellers_rpms[1]},{propellers_rpms[2]},{propellers_rpms[3]},{rollRad},{pitchRad},{yawRad},{v_s[0]},{v_s[1]},{v_s[2]},{v_s_d[0]},{v_s_d[1]},{v_s_d[2]},{rope_insideWindCount}");
            }else{
                tw.WriteLine($"{Time.time},{x_s[0]},{x_s[1]},{x_s[2]},{x_s_d[0]},{x_s_d[1]},{x_s_d[2]},{propellers_rpms[0]},{propellers_rpms[1]},{propellers_rpms[2]},{propellers_rpms[3]},{rollRad},{pitchRad},{yawRad},{v_s[0]},{v_s[1]},{v_s[2]},{v_s_d[0]},{v_s_d[1]},{v_s_d[2]},{rope_insideWindCount}");
            }
            tw.Close();
        }

        Vector<double> b1d = DenseVector.OfArray(new double[] { Math.Sqrt(2)/2, -Math.Sqrt(2)/2, 0 });

        // Control
        Vector<double> ex = x_s - x_s_d;
        Vector<double> ev = v_s - v_s_d;

        Vector<double> pid = -kx*ex - kv*ev + mQ*g*e3 + mQ*a_s_d;
        Vector<double> b3d = pid/pid.Norm(2);
        Vector<double> b2d = _Cross(b3d, b1d)/_Cross(b3d, b1d).Norm(2);
        Vector<double> b1d_temp = _Cross(b2d, b3d);
        Matrix<double> R_sb_d = DenseMatrix.OfArray(new double[,] { { b1d_temp[0], b2d[0], b3d[0] },
                                                                    { b1d_temp[1], b2d[1], b3d[1] },
                                                                    { b1d_temp[2], b2d[2], b3d[2] } });
        
        Vector<double> W_b_d = _Vee(_Logm3(R_sb_d_prev.Transpose()*R_sb_d)/dt);
        Vector<double> W_b_d_dot = (W_b_d - W_b_d_prev)/dt;

        Vector<double> eR = 0.5*_Vee(R_sb_d.Transpose()*R_sb - R_sb.Transpose()*R_sb_d);
        Vector<double> eW = W_b - R_sb.Transpose()*R_sb_d*W_b_d;

        f = pid*(R_sb*e3);
        M = -kR*eR - kW*eW + _Cross(W_b, J*W_b) - J*(_Hat(W_b)*R_sb.Transpose()*R_sb_d*W_b_d - R_sb.Transpose()*R_sb_d*W_b_d_dot);

        R_sb_d_prev = R_sb_d;
        W_b_d_prev = W_b_d;

        if (times2 < 2 || M.Norm(2) > 100) // If previous values have not been initialized yet or moments are excessive
        {
            times2++;
            f = 0;
            M = DenseVector.OfArray(new double[] { 0, 0, 0 });
        }
        // Debug.Log($"R_sb: {R_sb}, R_sb_d: {R_sb_d} R_sb_d_prev: {R_sb_d_prev}");
        return (f, M);
    }

	void ComputeRPMs() 
    {
        double f;
        Vector<double> M;

        // If rope has been replaced (tension is high enough) use suspended load controller
        // If we have not hooked the rope yet, use normal tracking controller
        if (Rope != null && Rope.childCount == 2) (f, M) = SuspendedLoadControl();
        else (f, M) = TrackingControl();

        // Compute optimal propeller forces
        Vector<double> F_star = T_inv * DenseVector.OfArray(new double[] { f, M[0], M[1], M[2] });

        // Build a matrix A and a vector b to solve for the variation on the optimal propeller forces
        Matrix<double> A = Matrix<double>.Build.Dense(NUM_PROPS, NUM_PROPS);
        Vector<double> b = Vector<double>.Build.Dense(NUM_PROPS);
        for (int i = 0; i < NUM_PROPS; i++) {
            for (int j = 0; j < NUM_PROPS; j++) {
                if (F_star[i] >= 0 && F_star[j] >= 0) {
                    A[i, j] = Q[i, j];
                } else if (i == j) {
                    A[i, j] = 1;
                } else {
                    A[i, j] = 0;
                }
                if (F_star[i] >= 0 && F_star[j] < 0) {
                    b[i] += Q[i, j]*F_star[j];
                }
            }
            if (F_star[i] < 0) {
                b[i] = -F_star[i];
            }
        }
        Vector<double> F = F_star + A.Solve(b);

        // If the gradient tangent to any of the boundaries makes the solution not satisfy the constraints,
        // project it back to the boundary
        for (int i = 0; i < NUM_PROPS; i++) {
            if (F[i] < 0) {
                F[i] = 0;
            }
        }

        // Set propeller rpms
        for (int i = 0; i < propellers.Length; i++)
            propellers_rpms[i] = (float)F[i]/propellers[i].RPMToForceMultiplier;
	}

	void ApplyRPMs() 
    {
        //Debug.Log($"RPM: {propellers_rpms[0]:F2},{propellers_rpms[1]:F2},{propellers_rpms[2]:F2},{propellers_rpms[3]:F2}"); // desired position

        //Debug.Log($"AUV: {LoadLinkTF.position[0]:F2},{LoadLinkTF.position[1]:F2},{LoadLinkTF.position[2]:F2}"); // 0, 0.04, 0.65

        Vector<double> x_s = BaseLink.transform.position.To<ENU>().ToDense();
        //Debug.Log($"UAV: {x_s[0]:F2},{x_s[1]:F2},{x_s[2]:F2}");    // -0.06, 2.94, 1.96

        //x_s = Rope.GetChild(Rope.childCount-1).position.To<ENU>().ToDense();
        //Debug.Log($"RopeCount: {Rope.childCount}");
        //Debug.Log($"Rope: {x_s[0]:F2},{x_s[1]:F2},{x_s[2]:F2}");    // 0, 2.75, 0
        //x_s = Rope.GetChild(7).position.To<ENU>().ToDense();
        //Debug.Log($"Rope10: {x_s[0]:F2},{x_s[1]:F2},{x_s[2]:F2}");    // 0, 1.65, 0     // 0, 0.65, 0

         // Rope has 20 child objects
        int numPoints = Rope.childCount;
        double[,] points = new double[numPoints, 3]; // 20 rows (points) and 3 columns (x, y, z)
        for (int i = 0; i < numPoints; i++)
        {
            // Get the position of the current child
            var position = Rope.GetChild(i).position.To<ENU>().ToDense();

            // Extract x, y, and z coordinates
            points[i, 0] = position[0];
            points[i, 1] = position[1];
            points[i, 2] = position[2];
        }       

        // Quadrotor roll pitch yaw
        Quaternion orientation = BaseLink.transform.rotation;
        Vector3 eulerENU = orientation.eulerAngles;

        double roll = eulerENU.x;
        double pitch = eulerENU.y;
        double yaw = eulerENU.z;

        // UAV attitude
        // double[] directionsScaled = {roll, pitch, yaw};      // 0,  45 , 0
        //double[] directionsScaled = {eulerENU.x, eulerENU.z, eulerENU.y};      // 0,  45 , 0
        // double[] directionsScaled = {1, 0, 0}; 
        // double[] directionsScaled = {eulerENU.y, eulerENU.z, eulerENU.x}; 

        //Quaternion adjustToXZPlane = Quaternion.Euler(90, 0, 0); // 90 degrees rotation around the X-axis 
        //Vector3 vectorDirectionScaled = orientation * adjustToXZPlane * Vector3.up; 

        Vector3 vectorDirectionScaled = orientation * Vector3.up; 

        //Debug.Log($"UAV : {x_s[0]:F2}, {x_s[1]:F2}, {x_s[2]:F2}");
        //Debug.Log($"eulerENU : {roll:F2}, {pitch:F2}, {yaw:F2}");
        //Debug.Log($"points : {numPoints}");
        
        //int insideCount = WindCheck.PointsInsideTrapezoid(points, directionsScaled, x_s[0], x_s[1], x_s[2]);
        //rope_insideWindCount = insideCount;        
        //Debug.Log($"Number of points inside the trapezoid: {insideCount}");

        // Plot downward windflow to exam 

        Vector3[] vectorPoints = ConvertToVector3Array(points);
        Gizmos_points = vectorPoints;
        
        //Vector3 vectorDirectionScaled = ConvertToVector3(directionsScaled);
        Vector3 UAV_position = ConvertVectorToVector3(x_s);
        // int PointsInsideTrapezoid(Vector3[] points, Vector3 directionScaled, Vector3 center)

        int insideCount2 = PointsInsideTrapezoid(vectorPoints,vectorDirectionScaled,UAV_position);
        rope_insideWindCount = insideCount2; 
        // Debug.Log($"2 Number of points inside the trapezoid: {insideCount2}");

        //Debug.DrawLine(UAV_position, UAV_position + vectorDirectionScaled * 5, Color.red, 2f);

        for (int i = 0; i < propellers.Length; i++) {
            // Now, all props should always have positive rpms, but just in case...
            if (propellers_rpms[i] < 0) {
                Debug.LogWarning("Propeller " + i + " has negative RPMs: " + propellers_rpms[i]);
                propellers_rpms[i] = 0;
            }
            propellers[i].SetRpm(propellers_rpms[i]);
        }
	}

    (Vector<double>, Vector<double>, Vector<double>) TrackingTargetTrajectory(Vector<double> x_TT, Vector<double> x_s, Vector<double> v_s) {
        Vector<double> x_s_d;
        Vector<double> v_s_d;
        Vector<double> a_s_d;
        
        Vector<double> unitVectorTowardsTarget = (x_TT - x_s)/(x_TT - x_s).Norm(2);
        // double accelerationDistance = Mathf.Pow(MaxVelocityWithTrackingTarget, 2)/(2*MaxAccelerationWithTrackingTarget);
        double distanceToTarget = (x_TT - x_s).Norm(2);
        // double velocityMagnitude;
        
        // // If we are not at the maximum velocity, we can accelerate
        // if (distanceToTarget > accelerationDistance && v_s.Norm(2) < MaxVelocityWithTrackingTarget) {
        //     if (startingPosition == null) {
        //         startingPosition = x_s - 1e-3*unitVectorTowardsTarget;
        //     }
        //     velocityMagnitude = Math.Sqrt(2*MaxAccelerationWithTrackingTarget*(x_s - startingPosition).Norm(2));
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        //     v_s_d = velocityMagnitude*unitVectorTowardsTarget;
        //     a_s_d = MaxAccelerationWithTrackingTarget*unitVectorTowardsTarget;
        // // If we want to move towards the target with maximum velocity
        // } else if (distanceToTarget > accelerationDistance) {
        //     startingPosition = null;
        //     velocityMagnitude = MaxVelocityWithTrackingTarget;
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        //     v_s_d = velocityMagnitude*unitVectorTowardsTarget;
        //     a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        // // If we are close to the target, slow down
        // } else if (distanceToTarget > 0.1) {
        //     startingPosition = null;
        //     velocityMagnitude = Math.Sqrt(2*MaxAccelerationWithTrackingTarget*distanceToTarget);
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        //     v_s_d = velocityMagnitude*unitVectorTowardsTarget;
        //     a_s_d = -MaxAccelerationWithTrackingTarget*unitVectorTowardsTarget;
        // // If we are at the target, stop
        // } else {
        //     startingPosition = null;
        //     x_s_d = x_TT;
        //     v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        //     a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        // }

        // // If we are not at the maximum velocity, we can accelerate
        // if (distanceToTarget > accelerationDistance && v_s.Norm(2) < MaxVelocityWithTrackingTarget) {
        //     if (startingPosition == null) {
        //         startingPosition = x_s - 1e-3*unitVectorTowardsTarget;
        //     }
        //     velocityMagnitude = Math.Sqrt(2*MaxAccelerationWithTrackingTarget*(x_s - startingPosition).Norm(2));
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        // // If we want to move towards the target with maximum velocity
        // } else if (distanceToTarget > accelerationDistance) {
        //     startingPosition = null;
        //     velocityMagnitude = MaxVelocityWithTrackingTarget;
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        // // If we are close to the target, slow down
        // } else if (distanceToTarget > 0.1) {
        //     startingPosition = null;
        //     velocityMagnitude = Math.Sqrt(2*MaxAccelerationWithTrackingTarget*distanceToTarget);
        //     x_s_d = x_s + velocityMagnitude*dt*unitVectorTowardsTarget;
        // // If we are at the target, stop
        // } else {
        //     startingPosition = null;
        //     x_s_d = x_TT;
        // }
        // v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        // a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });

        if (distanceToTarget > DecelerationDistance) {
            x_s_d = x_s + DecelerationDistance*unitVectorTowardsTarget;
            v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });//MaxVelocityWithTrackingTarget*unitVectorTowardsTarget;
            a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        } else {
            x_s_d = x_TT;
            v_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });//MaxVelocityWithTrackingTarget*distanceToTarget/DecelerationDistance*unitVectorTowardsTarget;
            a_s_d = DenseVector.OfArray(new double[] { 0, 0, 0 });
        }

        return (x_s_d, v_s_d, a_s_d);
    }

    (Vector<double>, Vector<double>, Vector<double>) Figure8Trajectory(double t) {
        double t_max = 20;
        double A = 8;
        double B = 4;
        double C = 2;
        double freq = 2 * Math.PI / t_max;

        Vector<double> x_s_d = DenseVector.OfArray(new double[] { A*Math.Sin(freq*t), B*Math.Sin(2*freq*t), C*Math.Sin(freq*t) + 6 });
        Vector<double> v_s_d = DenseVector.OfArray(new double[] { A*freq*Math.Cos(freq*t), 2*B*freq*Math.Cos(2*freq*t), C*freq*Math.Cos(freq*t) });
        Vector<double> a_s_d = DenseVector.OfArray(new double[] { -A*freq*freq*Math.Sin(freq*t), -4*B*freq*freq*Math.Sin(2*freq*t), -C*freq*freq*Math.Sin(freq*t) });

        return (x_s_d, v_s_d, a_s_d);
    }

    (Vector<double>, Vector<double>, Vector<double>, Vector<double>) HelixTrajectory(double t) {
        double t_max = 5;
        double A = 1;
        double B = 4;
        double C = 6;
        double freq = Math.PI / t_max;

        Vector<double> x_s_d = DenseVector.OfArray(new double[] { A*t, B*Math.Sin(freq*t),            C*Math.Cos(freq*t) + 10 });
        Vector<double> v_s_d = DenseVector.OfArray(new double[] { A,   B*freq*Math.Cos(freq*t),      -C*freq*Math.Sin(freq*t) });
        Vector<double> a_s_d = DenseVector.OfArray(new double[] { 0,  -B*freq*freq*Math.Sin(freq*t), -C*freq*freq*Math.Cos(freq*t)});

        Vector<double> b1d_d = DenseVector.OfArray(new double[] { Math.Cos(freq*t), Math.Sin(freq*t), 0 });
        return (x_s_d, v_s_d, a_s_d, b1d_d);
    }

    static Vector<double> _Cross(Vector<double> a, Vector<double> b) 
    {
        // Calculate each component of the cross product
        double c1 = a[1] * b[2] - a[2] * b[1];
        double c2 = a[2] * b[0] - a[0] * b[2];
        double c3 = a[0] * b[1] - a[1] * b[0];

        // Create a new vector for the result
        return DenseVector.OfArray(new double[] { c1, c2, c3 });
    }

    static Matrix<double> _Hat(Vector<double> v) 
    {
        return DenseMatrix.OfArray(new double[,] { { 0, -v[2], v[1] },
                                                   { v[2], 0, -v[0] },
                                                   { -v[1], v[0], 0 } });
    }
    
    static Vector<double> _Vee(Matrix<double> S) 
    {
        return DenseVector.OfArray(new double[] { S[2, 1], S[0, 2], S[1, 0] });
    }

    static Matrix<double> _Logm3(Matrix<double> R) 
    {
		double acosinput = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2.0;
		Matrix<double> m_ret = DenseMatrix.OfArray(new double[,] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } });
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Vector<double> omg;
			if (!(Math.Abs(1 + R[2, 2]) < 1e-6f))
				omg = (1.0 / Math.Sqrt(2 * (1 + R[2, 2])))*DenseVector.OfArray(new double[] { R[0, 2], R[1, 2], 1 + R[2, 2] });
			else if (!(Math.Abs(1 + R[1, 1]) < 1e-6f))
				omg = (1.0 / Math.Sqrt(2 * (1 + R[1, 1])))*DenseVector.OfArray(new double[] { R[0, 1], 1 + R[1, 1], R[2, 1] });
			else
				omg = (1.0 / Math.Sqrt(2 * (1 + R[0, 0])))*DenseVector.OfArray(new double[] { 1 + R[0, 0], R[1, 0], R[2, 0] });
			m_ret = _Hat(Math.PI * omg);
			return m_ret;
		}
		else {
			double theta = Math.Acos(acosinput);
			m_ret = theta / 2.0 / Math.Sin(theta)*(R - R.Transpose());
			return m_ret;
		}
	}

    static Vector3 ToUnity(Vector<double> v) 
    {
        return new Vector3((float)v[0], (float)v[2], (float)v[1]);
    }

    Vector3[] ConvertToVector3Array(double[,] doublePoints)
    {
        // Get the number of rows (outer dimension) of the double array
        int rowCount = doublePoints.GetLength(0);

        // Initialize the Vector3 array
        Vector3[] vectorPoints = new Vector3[rowCount];

        // Iterate through the rows and populate the Vector3 array
        for (int i = 0; i < rowCount; i++)
        {
            vectorPoints[i] = new Vector3(
                (float)doublePoints[i, 0], // Convert x to float
                (float)doublePoints[i, 2], // Convert y to float
                (float)doublePoints[i, 1]  // Convert z to float
            );
        }

        return vectorPoints;
    }

    Vector3 ConvertToVector3(double[] directionsScaled)
    {
        // Ensure the array has exactly 3 elements
        if (directionsScaled.Length != 3)
        {
            Debug.LogError("The double[] array must have exactly 3 elements to convert to a Vector3.");
            return Vector3.zero;
        }

        // Convert to Vector3
        return new Vector3(
            (float)directionsScaled[0], // x
            (float)directionsScaled[1], // y
            (float)directionsScaled[2]  // z
        );
    }

    Vector3 ConvertVectorToVector3(Vector<double> vectorDouble)
    {
        // Ensure the vector has exactly 3 elements
        if (vectorDouble.Count != 3)
        {
            Debug.LogError("The Vector<double> must have exactly 3 elements to convert to a Vector3.");
            return Vector3.zero;
        }

        // Convert to Vector3
        return new Vector3(
            (float)vectorDouble[0], // x
            (float)vectorDouble[2], // y
            (float)vectorDouble[1]  // z
        );
    }


    int PointsInsideTrapezoid(Vector3[] points, Vector3 directionScaled, Vector3 center)
    {
        float smallRadius = 0.3f;
        float largeRadius = 1.0f;
        float height = 2.5f;
        
        Vector3 v = directionScaled.normalized;
        //Vector3 v0 = Vector3.forward; // Original direction
        Vector3 v0 = -1*Vector3.right; // Original direction

        Vector3 axisOfRotation = Vector3.Cross(v0, v).normalized;
        //float angleOfRotation = Mathf.Acos(Vector3.Dot(v0, v));
        float angleOfRotation = Mathf.Acos(Mathf.Clamp(Vector3.Dot(v0, v), -1f, 1f));
        if (axisOfRotation == Vector3.zero)
        {
            axisOfRotation = Vector3.up; // Default axis when vectors are parallel
        }
        else
        {
            axisOfRotation.Normalize();
        }

        Quaternion rotation = Quaternion.AngleAxis(Mathf.Rad2Deg * angleOfRotation, axisOfRotation);

        List<Vector3> smallCircle = new List<Vector3>();
        List<Vector3> largeCircle = new List<Vector3>();

        for (int i = 0; i < 25; i++)
        {
            float theta = Mathf.PI * 2 * i / 25;
            Vector3 smallPoint = rotation * new Vector3(0, smallRadius * Mathf.Cos(theta), smallRadius * Mathf.Sin(theta));
            Vector3 largePoint = rotation * new Vector3(height, largeRadius * Mathf.Cos(theta), largeRadius * Mathf.Sin(theta));

            smallCircle.Add(smallPoint + center);
            largeCircle.Add(largePoint + center);
        }

        DrawTrapezoid(smallCircle, largeCircle);

        int count = 0;
        Vector3 vectorBetweenCircles = rotation * new Vector3(height, 0, 0);
        Vector3 unitVectorBetweenCircles = vectorBetweenCircles.normalized;

        foreach (Vector3 point in points)
        {
            Vector3 relativePoint = point - center;
            float projection = Vector3.Dot(relativePoint, unitVectorBetweenCircles);

            if (projection < 0 || projection > height)
                continue;

            float interpolatedRadius = Mathf.Lerp(smallRadius, largeRadius, projection / height);
            float distanceToAxis = Vector3.Cross(relativePoint, unitVectorBetweenCircles).magnitude;

            if (distanceToAxis <= interpolatedRadius)
                count++;
        }

        return count;
    }


    void DrawTrapezoid(List<Vector3> smallCircle, List<Vector3> largeCircle)
    {
        // for (int i = 0; i < smallCircle.Count; i++)
        // {
        //     Vector3 start = smallCircle[i];
        //     Vector3 end = largeCircle[i];
        //     Debug.DrawLine(start, end, Color.cyan, 1f);
        // }

        for (int i = 0; i < smallCircle.Count - 1; i++)
        {
            Debug.DrawLine(smallCircle[i], smallCircle[i + 1], Color.blue, 1f);
            Debug.DrawLine(largeCircle[i], largeCircle[i + 1], Color.green, 1f);
        }
    }

    // draw the rope points
    // void OnDrawGizmos()
    // {
    //     Gizmos.color = Color.red;
    //     foreach (var point in Gizmos_points)
    //     {
    //         Gizmos.DrawSphere(point, 0.1f);
    //     }
    // }


}



// Helper class for Minimum Snap Trajectory
public static class MinimumSnapTrajectory
{
    public static double[] MinimumSnapCoefficients(double startPos, double startVel, double startAcc,
                                                   double endPos, double endVel, double endAcc, 
                                                   double T)
    {
        var A = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            {1, 0, 0,    0,    0,    0},      
            {0, 1, 0,    0,    0,    0},      
            {0, 0, 2,    0,    0,    0},      
            {1, T, Math.Pow(T, 2), Math.Pow(T, 3), Math.Pow(T, 4), Math.Pow(T, 5)}, 
            {0, 1, 2*T,  3*Math.Pow(T, 2), 4*Math.Pow(T, 3), 5*Math.Pow(T, 4)},    
            {0, 0, 2,    6*T,  12*Math.Pow(T, 2), 20*Math.Pow(T, 3)}               
        });

        var b = Vector<double>.Build.Dense(new double[]
        {
            startPos, startVel, startAcc, endPos, endVel, endAcc
        });

        var x = A.Solve(b);

        return x.ToArray();
    }

    // Evaluate the polynomial at a given time t
    public static double EvaluatePolynomial(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 0; i < coeffs.Length; i++)
        {
            result += coeffs[i] * Math.Pow(t, i);
        }
        return result;
    }

    // Evaluate the first derivative (velocity) of the polynomial at time t
    public static double EvaluatePolynomialDerivative(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 1; i < coeffs.Length; i++)  // Start at i=1 because the derivative of a0 is 0
        {
            result += i * coeffs[i] * Math.Pow(t, i - 1);
        }
        return result;
    }

    // Evaluate the second derivative (acceleration) of the polynomial at time t
    public static double EvaluatePolynomialSecondDerivative(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 2; i < coeffs.Length; i++)  // Start at i=2 because the second derivative of a0 and a1 is 0
        {
            result += i * (i - 1) * coeffs[i] * Math.Pow(t, i - 2);
        }
        return result;
    }
}

public static class ContinueMinimumSnapTrajectory
{
    // Calculate the coefficients for a single trajectory segment
    public static double[] MinimumSnapCoefficients(double startPos, double startVel, double startAcc,
                                                   double endPos, double endVel, double endAcc, 
                                                   double T)
    {
        var A = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            {1, 0, 0,    0,    0,    0},      
            {0, 1, 0,    0,    0,    0},      
            {0, 0, 2,    0,    0,    0},      
            {1, T, Math.Pow(T, 2), Math.Pow(T, 3), Math.Pow(T, 4), Math.Pow(T, 5)}, 
            {0, 1, 2*T,  3*Math.Pow(T, 2), 4*Math.Pow(T, 3), 5*Math.Pow(T, 4)},    
            {0, 0, 2,    6*T,  12*Math.Pow(T, 2), 20*Math.Pow(T, 3)}               
        });

        var b = Vector<double>.Build.Dense(new double[]
        {
            startPos, startVel, startAcc, endPos, endVel, endAcc
        });

        var x = A.Solve(b);

        return x.ToArray();
    }

    // Multi-segment trajectory coefficients generator
    public static List<double[]> GenerateTrajectory(List<double> positions, List<double> velocities, 
                                                    List<double> accelerations, List<double> times)
    {
        if (positions.Count != times.Count || positions.Count < 2)
            throw new ArgumentException("Positions and times must have the same length and at least 2 points.");

        List<double[]> trajectoryCoefficients = new List<double[]>();

        for (int i = 0; i < positions.Count - 1; i++)
        {
            double T = times[i + 1] - times[i];
            var coeffs = MinimumSnapCoefficients(
                positions[i], velocities[i], accelerations[i],
                positions[i + 1], velocities[i + 1], accelerations[i + 1], T
            );
            trajectoryCoefficients.Add(coeffs);
        }

        return trajectoryCoefficients;
    }

    // Evaluate trajectory for a given time t
    public static (double position, double velocity, double acceleration) EvaluateTrajectory(
        List<double[]> coefficients, List<double> times, double t)
    {
        if (t < times[0] || t > times[^1])
            throw new ArgumentException("Time t is out of bounds for the trajectory.");

        // Determine which segment to use
        int segment = times.Count - 2; // Default to last segment
        for (int i = 0; i < times.Count - 1; i++)
        {
            if (t >= times[i] && t < times[i + 1])
            {
                segment = i;
                break;
            }
        }

        double localTime = t - times[segment];
        var coeffs = coefficients[segment];

        double position = EvaluatePolynomial(coeffs, localTime);
        double velocity = EvaluatePolynomialDerivative(coeffs, localTime);
        double acceleration = EvaluatePolynomialSecondDerivative(coeffs, localTime);

        return (position, velocity, acceleration);
    }    

    // Evaluate the polynomial at a given time t
    public static double EvaluatePolynomial(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 0; i < coeffs.Length; i++)
        {
            result += coeffs[i] * Math.Pow(t, i);
        }
        return result;
    }

    // Evaluate the first derivative (velocity) of the polynomial at time t
    public static double EvaluatePolynomialDerivative(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 1; i < coeffs.Length; i++) // Start at i=1 because the derivative of a0 is 0
        {
            result += i * coeffs[i] * Math.Pow(t, i - 1);
        }
        return result;
    }

    // Evaluate the second derivative (acceleration) of the polynomial at time t
    public static double EvaluatePolynomialSecondDerivative(double[] coeffs, double t)
    {
        double result = 0;
        for (int i = 2; i < coeffs.Length; i++) // Start at i=2 because the second derivative of a0 and a1 is 0
        {
            result += i * (i - 1) * coeffs[i] * Math.Pow(t, i - 2);
        }
        return result;
    }

    // Existing methods are unchanged.

    // Calculate the coefficients for a single trajectory segment with soft constraints
    public static double[] MinimumSnapSoftConstraints(double startPos, double startVel, double startAcc,
                                                      double endPos, double endVel, double endAcc, 
                                                      double T, double lambda)
    {
        int degree = 5; // Degree of the polynomial (quintic)
        int numCoeffs = degree + 1;

        // Objective matrix for snap minimization (Q)
        var Q = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            {0, 0, 0, 0, 0, 0}, 
            {0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0},
            {0, 0, 0, 36, 0, 0},
            {0, 0, 0, 0, 192, 0},
            {0, 0, 0, 0, 0, 720},
        });
        Q *= T;

        // Constraints (A_eq * x = b_eq)
        var A_eq = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            {1, 0, 0,    0,    0,    0},      
            {0, 1, 0,    0,    0,    0},      
            {0, 0, 2,    0,    0,    0},      
            {1, T, Math.Pow(T, 2), Math.Pow(T, 3), Math.Pow(T, 4), Math.Pow(T, 5)}, 
            {0, 1, 2*T,  3*Math.Pow(T, 2), 4*Math.Pow(T, 3), 5*Math.Pow(T, 4)},    
            {0, 0, 2,    6*T,  12*Math.Pow(T, 2), 20*Math.Pow(T, 3)}               
        });
        var b_eq = Vector<double>.Build.Dense(new double[] { startPos, startVel, startAcc, endPos, endVel, endAcc });

        // Soft constraints weights (lambda controls how strongly soft constraints are enforced)
        var P = Q + lambda * Matrix<double>.Build.DenseIdentity(numCoeffs);

        // Solve the quadratic program: minimize (1/2) * x^T * P * x subject to A_eq * x = b_eq
        var qpSolver = new GoldfarbIdnani(
            P.ToArray(), 
            Vector<double>.Build.Dense(numCoeffs).ToArray(),
            A_eq.ToArray(), 
            b_eq.ToArray());

        bool success = qpSolver.Minimize();

        if (!success)
            throw new Exception("Quadratic programming solver failed!");

        return qpSolver.Solution;
    }

    // Overload for multi-segment trajectory generation with soft constraints
    public static List<double[]> GenerateTrajectorySoft(List<double> positions, List<double> velocities, 
                                                        List<double> accelerations, List<double> times,
                                                        double lambda)
    {
        if (positions.Count != times.Count || positions.Count < 2)
            throw new ArgumentException("Positions and times must have the same length and at least 2 points.");

        List<double[]> trajectoryCoefficients = new List<double[]>();

        for (int i = 0; i < positions.Count - 1; i++)
        {
            double T = times[i + 1] - times[i];
            var coeffs = MinimumSnapSoftConstraints(
                positions[i], velocities[i], accelerations[i],
                positions[i + 1], velocities[i + 1], accelerations[i + 1], T, lambda
            );
            trajectoryCoefficients.Add(coeffs);
        }

        return trajectoryCoefficients;
    }    
}

// Examine the impact of the UAV's downward airflow on the rope
// Usage: 
        // Rope Locations

        // double[,] points = {
        //     { 1, 0, 0 },
        //     { 3, 0, 7 },
        //     { 0, 0, 20 },
        //     { -1, -1, 5 }
        // };

        // double[] directionsScaled = { 1, 0, 0 };       // UAV attitude
        // double x = 0, y = 0, z = 0;                    // UAV position     

        // int insideCount = WindCheck.PointsInsideTrapezoid(points, directionsScaled, x, y, z);
        // Console.WriteLine($"Number of points inside the trapezoid: {insideCount}");

public static class WindCheck
{
    public static int PointsInsideTrapezoid(double[,] points, double[] directionsScaled, double x, double y, double z)
    {
        double r = 2;    // UAV's downward airflow range is assumed as a trapezoid
        double R = 5;
        double h = 1;
        // int thetaResolution = 25;

        // // Create theta array
        // double[] theta = Enumerable.Range(0, thetaResolution)
        //                            .Select(i => 2 * Math.PI * i / (thetaResolution - 1))
        //                            .ToArray();

        // Normalize the direction vector
        double[] v = NormalizeVector(directionsScaled);
        double[] v0 = { 0, 0, 1 }; // Original direction (along x-axis)

        // Calculate the axis of rotation
        double[] axisOfRotation = NormalizeVector(CrossProduct(v0, v));

        // Calculate the angle of rotation
        double angleOfRotation = Math.Acos(DotProduct(v0, v));

        // Rotation matrix using Rodrigues' rotation formula
        double[,] K = {
            { 0, -axisOfRotation[2], axisOfRotation[1] },
            { axisOfRotation[2], 0, -axisOfRotation[0] },
            { -axisOfRotation[1], axisOfRotation[0], 0 }
        };

        double[,] RMatrix = AddMatrices(
            IdentityMatrix(3),
            MultiplyMatrixByScalar(K, Math.Sin(angleOfRotation)),
            MultiplyMatrixByScalar(PowerMatrix(K, 2), 1 - Math.Cos(angleOfRotation))
        );

        // Circle centers
        double[] circle2Center = AddVectors(
            MultiplyMatrixByVector(RMatrix, new double[] { h, 0, 0 }),
            new double[] { x, y, z }
        );

        double[] vectorBetweenCircles = MultiplyMatrixByVector(RMatrix, new double[] { h, 0, 0 });
        double[] vectorNorm = NormalizeVector(vectorBetweenCircles);

        // Check points
        int count = 0;
        for (int i = 0; i < points.GetLength(0); i++)
        {
            double[] point = { points[i, 0], points[i, 1], points[i, 2] };

            double[] relativePoint = SubtractVectors(point, new double[] { x, y, z });
            double projection = DotProduct(relativePoint, vectorNorm);

            if (projection < 0 || projection > h)
                continue;

            double interpolatedRadius = r + (R - r) * (projection / h);

            double distanceToAxis = Norm(CrossProduct(relativePoint, vectorNorm)) / Norm(vectorNorm);

            if (distanceToAxis <= interpolatedRadius)
                count++;
        }

        return count;
    }


    // Utility Functions
    private static double[] NormalizeVector(double[] v)
    {
        double norm = Math.Sqrt(v.Sum(x => x * x));
        return v.Select(x => x / norm).ToArray();
    }

    private static double[] CrossProduct(double[] u, double[] v)
    {
        return new double[]
        {
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]
        };
    }

    private static double DotProduct(double[] u, double[] v)
    {
        return u.Zip(v, (a, b) => a * b).Sum();
    }

    private static double[,] IdentityMatrix(int size)
    {
        double[,] identity = new double[size, size];
        for (int i = 0; i < size; i++) identity[i, i] = 1;
        return identity;
    }

    private static double[,] AddMatrices(params double[][,] matrices)
    {
        int rows = matrices[0].GetLength(0), cols = matrices[0].GetLength(1);
        double[,] result = new double[rows, cols];

        foreach (var matrix in matrices)
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result[i, j] += matrix[i, j];

        return result;
    }

    private static double[,] MultiplyMatrixByScalar(double[,] matrix, double scalar)
    {
        int rows = matrix.GetLength(0), cols = matrix.GetLength(1);
        double[,] result = new double[rows, cols];

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[i, j] = matrix[i, j] * scalar;

        return result;
    }

    private static double[] MultiplyMatrixByVector(double[,] matrix, double[] vector)
    {
        int rows = matrix.GetLength(0), cols = matrix.GetLength(1);
        double[] result = new double[rows];

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result[i] += matrix[i, j] * vector[j];

        return result;
    }

    private static double[] SubtractVectors(double[] u, double[] v)
    {
        return u.Zip(v, (a, b) => a - b).ToArray();
    }

    private static double[] AddVectors(double[] u, double[] v)
    {
        return u.Zip(v, (a, b) => a + b).ToArray();
    }

    private static double Norm(double[] v)
    {
        return Math.Sqrt(v.Sum(x => x * x));
    }

    private static double[,] PowerMatrix(double[,] matrix, int power)
    {
        double[,] result = IdentityMatrix(matrix.GetLength(0));
        for (int i = 0; i < power; i++)
        {
            result = MultiplyMatrices(result, matrix);
        }
        return result;
    }

    private static double[,] MultiplyMatrices(double[,] a, double[,] b)
    {
        int rowsA = a.GetLength(0), colsA = a.GetLength(1), colsB = b.GetLength(1);
        double[,] result = new double[rowsA, colsB];

        for (int i = 0; i < rowsA; i++)
            for (int j = 0; j < colsB; j++)
                for (int k = 0; k < colsA; k++)
                    result[i, j] += a[i, k] * b[k, j];

        return result;
    }
}

