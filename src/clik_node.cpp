#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sun_robot_lib/Robots/LBRiiwa7.h"

bool jointStateArrived = false;
sensor_msgs::JointState jointState;
void readJointCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  jointState = *msg;
  jointStateArrived = true;
}

geometry_msgs::PoseStamped poseCMD;
void readCommandPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  poseCMD = *msg;
}

geometry_msgs::TwistStamped twistCMD;
void readCommandVelCB(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  twistCMD = *msg;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "clik");

  ros::NodeHandle nh;

  sun::LBRiiwa7 iiwa;  // oggetto robot

  // Questo parametro influenza la pinv DLS e riduce le velocità massime di giunto
  // Il parametro di ingresso NON è una vera saturazione ma è euristicamente legato alle massime velocità di giunto che
  // usciranno dalla pinv_DLS attualente non esiste una relazione in forma chiusa per legare questo parametro alle vere
  // massime velocità di giunto che usciranno dalla pinv ovviamente questo parametro può anche influenzare la stabilità
  // del clik deve essere diverso da 0
  iiwa.setDLSJointSpeedSaturation(5.0);

  // Dove pubblicare i comandi? ovviamente il publisher è diverso a seconda dei casi
  ros::Publisher pub_joints = nh.advertise<sensor_msgs::JointState>("/iiwa/command/joint_states", 1);

  // Dove leggere i comandi?
  ros::Subscriber sub_command_pose = nh.subscribe("/iiwa/command/cartesian_position", 1, readCommandPoseCB);
  ros::Subscriber sub_command_vel = nh.subscribe("/iiwa/command/cartesian_velocity", 1, readCommandVelCB);

  // Devo inizializzare il clik allo stato iniziale del robot
  ros::Subscriber sub_joints = nh.subscribe("/iiwa/joint_states", 1, readJointCB);

  // Leggi la configurazione iniziale del robot per inizializzare il clik allo stato iniziale q0
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

  ROS_INFO_STREAM("Initial Robot Conf:\n" << qR);

  // Inizializza il messaggio di uscita (il tipo di messaggio può essere diverso a seconda dei casi)
  sensor_msgs::JointState commandMsg;
  commandMsg.name = jointState.name;
  commandMsg.position.resize(iiwa.getNumJoints());
  commandMsg.velocity.resize(iiwa.getNumJoints());

  // Inizializzazione variabili
  TooN::Vector<> qDH_k = iiwa.joints_Robot2DH(qR);
  TooN::Vector<> qpDH = TooN::Zeros(iiwa.getNumJoints());
  sun::UnitQuaternion oldQ; // <-- quaternione al passo precedente (serve per la continuità del quaternione)

  // Inizializziamo alcune variabili di supporto
  // Ci assicuriamo che l'errore iniziale sia nullo
  // (supponiamo che alla prima iterazione potrebbe non essere arrivato nessun comando)
  {
    TooN::Matrix<4, 4> T_init = iiwa.fkine(qDH_k);
    oldQ = sun::UnitQuaternion(T_init); 
    TooN::Vector<3> initialPosition = sun::transl(T_init);
    sun::UnitQuaternion initialQuaternion(T_init);
    poseCMD.pose.position.x = initialPosition[0];
    poseCMD.pose.position.y = initialPosition[1];
    poseCMD.pose.position.z = initialPosition[2];
    poseCMD.pose.orientation.w = initialQuaternion.getS();
    poseCMD.pose.orientation.x = initialQuaternion.getV()[0];
    poseCMD.pose.orientation.y = initialQuaternion.getV()[1];
    poseCMD.pose.orientation.z = initialQuaternion.getV()[2];
    twistCMD.twist.linear.x = 0.0;
    twistCMD.twist.linear.y = 0.0;
    twistCMD.twist.linear.z = 0.0;
    twistCMD.twist.angular.x = 0.0;
    twistCMD.twist.angular.y = 0.0;
    twistCMD.twist.angular.z = 0.0;
  }

  double hz = 1000.0;
  ros::Rate loop_rate(hz);

  // MAIN LOOP
  while (ros::ok())
  {
    // Aggiorno il punto attuale della traiettoria
    ros::spinOnce();
    TooN::Vector<3> pos = TooN::makeVector(poseCMD.pose.position.x, poseCMD.pose.position.y, poseCMD.pose.position.z);
    TooN::Vector<3> vel = TooN::makeVector(twistCMD.twist.linear.x, twistCMD.twist.linear.y, twistCMD.twist.linear.z);
    sun::UnitQuaternion quat(
        poseCMD.pose.orientation.w,
        TooN::makeVector(poseCMD.pose.orientation.x, poseCMD.pose.orientation.y, poseCMD.pose.orientation.z));
    TooN::Vector<3> w = TooN::makeVector(twistCMD.twist.angular.x, twistCMD.twist.angular.y, twistCMD.twist.angular.z);

    TooN::Vector<6> clik_error;
    // applico il clik
    qDH_k = iiwa.clik(qDH_k,       //<- qDH attuale
                      pos,         // <- posizione desiderata
                      quat,        // <- quaternione desiderato
                      oldQ,        // <- quaternione al passo precedente (per garantire la continuità)
                      vel,         // <- velocità in translazione desiderata
                      w,           //<- velocità angolare deisderata
                      TooN::Ones,  // <- maschera, se l'i-esimo elemento è zero allora l'i-esima componente cartesiana
                                   // non verrà usata per il calcolo dell'errore
                      0.2 * hz,    // <- guadagno del clik (quì è scelto in maniera conservativa)
                      1.0 / hz,    // <- Ts, tempo di campionamento
                      0.0,         // <- quadagno obj secondario
                      TooN::Zeros(iiwa.getNumJoints()),  // velocità di giunto dell'obj secondario (qui sono zero)
                      // Return Vars
                      qpDH,        // <- variabile di ritorno velocità di giunto
                      clik_error,  //<- variabile di ritorno errore
                      oldQ  // <- variabile di ritorno: Quaternione attuale (N.B. qui uso oldQ in modo da aggiornare
                            // direttamente la variabile oldQ e averla già pronta per la prossima iterazione)
    );

    // Calcolo q in robot convention
    qR = iiwa.joints_DH2Robot(qDH_k);

    // check limits
    if (iiwa.exceededHardJointLimits(qR))
    {
      // stampo a schermo i giunti incriminati
      ROS_ERROR_STREAM("ERROR ROBOT JOINT LIMITS!! On joints:\n"
                       << iiwa.jointsNameFromBitMask(iiwa.checkHardJointLimits(qR)));
      exit(-1);  // esco
    }
    if (iiwa.exceededHardVelocityLimits(iiwa.jointsvel_DH2Robot(qpDH)))
    {
      // stampo a schermo i giunti incriminati
      ROS_ERROR_STREAM("ERROR ROBOT Velocity!! On joints:\n"
                       << iiwa.jointsNameFromBitMask(iiwa.checkHardVelocityLimits(iiwa.jointsvel_DH2Robot(qpDH))));
      exit(-1);  // esco
    }

    // publish: in questo esempio il tipo di messaggio è sensor_msgs::JointState
    // ovviamente questa parte deve essere diversa a seconda del caso....
    for (int i = 0; i < iiwa.getNumJoints(); i++)
    {
      commandMsg.position[i] = qR[i];
      commandMsg.velocity[i] = qpDH[i];
    }
    pub_joints.publish(commandMsg);

    // sleep sul ros::Rate
    loop_rate.sleep();
  }

  return 0;
}
