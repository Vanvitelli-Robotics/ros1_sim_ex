#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sun_traj_lib/Quintic_Poly_Traj.h"
#include "sun_traj_lib/Vector_Independent_Traj.h"

bool jointStateArrived = false;
sensor_msgs::JointState jointState;
void readJointCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointState = *msg;
  jointStateArrived = true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "iiwa_go_to");

  ros::NodeHandle nh;

  // Dove pubblicare?
  ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("/iiwa/command/joint_states", 1);

  // Posizione iniziale della traiettoria
  ros::Subscriber sub_joints = nh.subscribe("/iiwa/joint_states", 1, readJointCB);

  double duration = 10.0;  // seconds
  TooN::Vector<> qRf = TooN::makeVector(0.0, M_PI / 4.0, 0.0, -M_PI / 2.0, 0.0, -M_PI / 4.0, 0.0);

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

  // Inizializza il messaggio di uscita (il tipo di messaggio può essere diverso a seconda dei casi)
  sensor_msgs::JointState commandMsg;
  commandMsg.name = jointState.name;
  commandMsg.position.resize(qR.size());
  commandMsg.velocity.resize(qR.size());

  // Costruisco la traiettoria in giunti come la traiettoria di un vettore in R^n dove ogni
  // componente evolve come un polinomio quintico
  sun::Vector_Independent_Traj traj;
  for (int i = 0; i < qR.size(); i++)
  {
    traj.push_back_traj(sun::Quintic_Poly_Traj(duration,  // duration
                                               qR[i],     // double initial_position,
                                               qRf[i]     // double final_position,
                                                          /*
                                                          double initial_time = 0.0,
                                                          double initial_velocity = 0.0,
                                                          double final_velocity = 0.0,
                                                          double initial_acceleration = 0.0,
                                                          double final_acceleration = 0.0
                                                          );*/

                                               ));
  }

  ROS_INFO_STREAM("START!");

  double hz = 100.0;
  ros::Rate loop_rate(hz);

  double time_now = ros::Time::now().toSec();

  // questo metodo cambia il tempo iniziale (tempo di start della traiettoria)
  // qui è usato passandogli il tempo attuale, ovvero stiamo dicendo: "la traiettoria inizia ora"
  traj.changeInitialTime(time_now);  //<-- il tempo iniziale della traiettoria è NOW

  // finchè (ros è ok) && ( contemporaneamente la traiettoria non è completata e l'errore non è nullo )
  while (ros::ok() && !traj.isCompleate(time_now))
  {
    // aggiorno il tempo attuale
    ros::Time ros_time_now = ros::Time::now();
    time_now = ros_time_now.toSec();

    ROS_INFO_STREAM_THROTTLE(0.5, "Time Left: " << traj.getTimeLeft(time_now));

    // prendo il punto attuale della traiettoria
    TooN::Vector<> pos = traj.getPosition(time_now);
    TooN::Vector<> vel = traj.getVelocity(time_now);

    commandMsg.header.stamp = ros_time_now;
    for (int i = 0; i < pos.size(); i++)
    {
      commandMsg.position[i] = pos[i];
      commandMsg.velocity[i] = vel[i];
    }

    pub_joints.publish(commandMsg);

    // sleep sul ros::Rate
    loop_rate.sleep();
  }

  return 0;
}
