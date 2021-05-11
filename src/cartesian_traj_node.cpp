#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sun_traj_lib/Cartesian_Independent_Traj.h"
#include "sun_traj_lib/Line_Segment_Traj.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Rotation_Const_Axis_Traj.h"

TooN::Matrix<4, 4> getTinit();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cartesian_traj");

  ros::NodeHandle nh;

  // Dove pubblicare?
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/command/cartesian_position", 1);
  ros::Publisher pub_twist = nh.advertise<geometry_msgs::TwistStamped>("/iiwa/command/cartesian_twist", 1);

  // Posa iniziale della traiettoria, in una implementazione seria dovrebbe essere sincronizzata tramite rosaction
  TooN::Matrix<4, 4> T_init = getTinit();

  // Polinomio quintico dell'ascissa curvilinea (deve essere tra 0 e 1)
  // sarà usato nella traiettoria in posizione
  sun::Quintic_Poly_Traj s_pos(10.0,  // duration
                               0.0,   // double initial_position,
                               1.0);  // double final_position,
  /*
  double initial_time = 0.0,
  double initial_velocity = 0.0,
  double final_velocity = 0.0,
  double initial_acceleration = 0.0,
  double final_acceleration = 0.0
  );*/

  // Traiettoria in posizione, NOTA: il costruttore vuole in ingresso anche un generatore di traiettoria per l'ascissa
  // curvilinea (vedi s_pos) Questa traiettoria è un segmento da pi a pf
  // NOTA: quì la posizione finale è data come posizione_iniziale+deltaP
  sun::Line_Segment_Traj lin_traj(sun::transl(T_init),                                      // pi
                                  sun::transl(T_init) + TooN::makeVector(0.0, 0.05, 0.25),  // pf
                                  s_pos  // const Scalar_Traj_Interface& traj_s
  );

  // Polinomio quintico per l'angolo, va da angolo iniziale a angolo finale (in questo caso da 0 a 45)
  // Nota: l'angolo iniziale è 0 l'angolo finale è un delta_angolo
  sun::Quintic_Poly_Traj s_rot(10.0,                  // duration
                               0.0,                   // double initial_position,
                               45.0 * M_PI / 180.0);  // double final_position,
  /*
  double initial_time = 0.0,
  double initial_velocity = 0.0,
  double final_velocity = 0.0,
  double initial_acceleration = 0.0,
  double final_acceleration = 0.0
  );*/

  // Traiettoria in rotazione
  // E' una traiettoria con asse di rotazione costante
  // vuole in ingresso: quaternione iniziale
  //                   asse di rotazione
  //                   traiettoria per l'angolo (in questo caso s_rot = oggetto precedente)
  sun::Rotation_Const_Axis_Traj rot_traj(sun::UnitQuaternion(T_init),      // initial_quat,
                                         TooN::makeVector(0.0, 0.0, 1.0),  // axis, (asse z in questo caso)
                                         s_rot);

  // Traiettoria completa in cartesiano
  // Questo oggetto è un wrapper che contiene le due traiettorie indipendenti in posizione e rotazione
  // la durata totale sarà la durata massima tra le due traiettorie (posizione e rotazione) (perchè una delle due
  // traiettorie potrebbe finire prima)
  sun::Cartesian_Independent_Traj cart_traj(lin_traj, rot_traj);

  ROS_INFO_STREAM("START!");

  double hz = 100.0;
  ros::Rate loop_rate(hz);

  double time_now = ros::Time::now().toSec();

  // questo metodo cambia il tempo iniziale (tempo di start della traiettoria)
  // qui è usato passandogli il tempo attuale, ovvero stiamo dicendo: "la traiettoria inizia ora"
  cart_traj.changeInitialTime(time_now);  //<-- il tempo iniziale della traiettoria è NOW

  // finchè (ros è ok) && ( contemporaneamente la traiettoria non è completata e l'errore non è nullo )
  while (ros::ok() && !cart_traj.isCompleate(time_now))
  {
    // aggiorno il tempo attuale
    ros::Time ros_time_now = ros::Time::now();
    time_now = ros_time_now.toSec();

    ROS_INFO_STREAM_THROTTLE(0.5, "Time Left: " << cart_traj.getTimeLeft(time_now));

    // prendo il punto attuale della traiettoria
    TooN::Vector<3> pos = cart_traj.getPosition(time_now);
    TooN::Vector<3> vel = cart_traj.getLinearVelocity(time_now);
    sun::UnitQuaternion quat = cart_traj.getQuaternion(time_now);
    TooN::Vector<3> w = cart_traj.getAngularVelocity(time_now);

    geometry_msgs::PoseStamped msgPose;
    msgPose.header.stamp = ros_time_now;
    msgPose.pose.position.x = pos[0];
    msgPose.pose.position.y = pos[1];
    msgPose.pose.position.z = pos[2];
    msgPose.pose.orientation.w = quat.getS();
    msgPose.pose.orientation.x = quat.getV()[0];
    msgPose.pose.orientation.y = quat.getV()[1];
    msgPose.pose.orientation.z = quat.getV()[2];

    geometry_msgs::TwistStamped msgTwist;
    msgTwist.header.stamp = ros_time_now;
    msgTwist.twist.linear.x = vel[0];
    msgTwist.twist.linear.y = vel[1];
    msgTwist.twist.linear.z = vel[2];
    msgTwist.twist.angular.x = w[0];
    msgTwist.twist.angular.y = w[1];
    msgTwist.twist.angular.z = w[2];

    pub_pose.publish(msgPose);
    pub_twist.publish(msgTwist);

    // sleep sul ros::Rate
    loop_rate.sleep();
  }

  return 0;
}

/*
    CODICE PER LEGGERE LA Tinit dai giunti del robot
    in una implementazione seria questo nodo dovrebbe essere una rosaction
    e il punto iniziale della traiettoria dovrebbe essere un parametro
*/

#include "sensor_msgs/JointState.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"

bool jointStateArrived = false;
sensor_msgs::JointState jointState;
void readJointCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointState = *msg;
  jointStateArrived = true;
}

TooN::Matrix<4, 4> getTinit()
{
  ros::NodeHandle nh;
  ros::Subscriber sub_joints = nh.subscribe("/iiwa/joint_states", 1, readJointCB);

  sun::LBRiiwa7 iiwa;

  jointStateArrived = false;
  while (ros::ok() && !jointStateArrived)
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  sub_joints.shutdown();
  TooN::Vector<> qR = TooN::Zeros(jointState.position.size());
  for (int i = 0; i < jointState.position.size(); i++)
    qR[i] = jointState.position[i];

  return iiwa.fkine(iiwa.joints_Robot2DH(qR));
}