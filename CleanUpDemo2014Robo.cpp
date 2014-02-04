#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class DemoRobotController : public Controller {
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

	void stopRobotMove(void);
	double goToObj(Vector3d pos, double range);
	double rotateTowardObj(Vector3d pos);
	void recognizeObjectPosition(Vector3d &pos, std::string &name);
	void throwTrash(void);
	double goGraspingObject(Vector3d &pos);
	void neutralizeArms(double evt_time);
	void prepareThrowing(double evt_time);

private:
	RobotObj *m_robotObject;
	ViewService *m_view;

	int m_state; 
	double refreshRateOnAction;

	std::string m_trashName1;
	std::string m_trashName2;
	std::string m_graspObjectName;

	std::string m_trashBoxName1;
	std::string m_trashBoxName2;

  double m_angularVelocity;  // rotation speed of the wheel
  double m_jointVelocity;    // rotation speed around the joint
  double m_radius;           // radius of the wheel
  double m_distance;         // length of wheel-track
  double m_movingSpeed;      // actual velocity of the moving robot

  // times wasted for moving, adjusting or driving
  double m_time;
  double m_time1;
  double m_time4;

	//positions
	Vector3d m_frontTrashBox1;
	Vector3d m_frontTrashBox2;
	Vector3d m_relayPoint1;
	Vector3d m_frontTrash1;
	Vector3d m_frontTrash2;

	// condition flag for grasping trash
	bool m_grasp;

  // angular parameter used to put robot's hands down
  double thetaA;
};


void DemoRobotController::onInit(InitEvent &evt) {
	// get robot's name
	m_robotObject = getRobotObj(myname());

	// set wheel configuration
	m_radius = 10.0;
	m_distance = 10.0;
	m_robotObject->setWheel(m_radius, m_distance);

	m_time = 0.0;
	m_time1 = 0.0;
	m_time4 = 0.0;

	m_state = 10;  // switch of initial behavior
	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

	// angular velocity of wheel and moving speed of robot
	m_angularVelocity = 1.5;
	m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)

	// rotation speed of joint
	m_jointVelocity = 0.5;

	m_trashName1 = "petbottle_1";
	m_trashName2 = "can_0";

	m_trashBoxName1 = "trashbox_0";  // for recycle
	m_trashBoxName2 = "trashbox_1";  // for burnable

	// set positions;
	m_frontTrashBox1 = Vector3d(-80.0, 0.0, -90);  // for recycle material
	m_frontTrashBox2 = Vector3d(20.0, 0.0, -90);  // for burnable material
	m_relayPoint1    = Vector3d(190.0, 0.0, -65.0);
	m_frontTrash1     = Vector3d(273.0, 0.0, -65.0);
	m_frontTrash2     = Vector3d(305.0, 0.0, -80.0);

	m_grasp = false;
}


double DemoRobotController::onAction(ActionEvent &evt) {
	switch(m_state){
		case 0: {
			break;
		}
		case 1: {
			this->stopRobotMove();
			break;
		}
		case 10: {  // go straight a bit
			m_graspObjectName = m_trashName2;  // at first, focusing to m_trashName2:can_0
			m_robotObject->setWheelVelocity(m_angularVelocity, m_angularVelocity);
			m_time = 10.0/m_movingSpeed + evt.time();  // time to be elapsed
			m_state = 20;
			break;
		}
		case 20: {  // direct to the trash
			if(evt.time() >= m_time && m_state==20){
				stopRobotMove();    // at first, stop robot maneuver

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);  // get position of trash
				double l_moveTime = rotateTowardObj(l_tpos);  // rotate toward the position and calculate the time to be elapsed.

				m_time = l_moveTime+evt.time();
				m_state = 30;
			}
			break;
		}
		case 30: {  // proceed toward trash
			if(evt.time() >= m_time && m_state==30){
				this->stopRobotMove();

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = goToObj(l_tpos, 75.0);  // go toward the position and calculate the time to be elapsed.

				m_time = l_moveTime+evt.time();
				m_state = 40;
      }
      break;
    }
		case 40: {  // get back a bit after colliding with the table
			if(evt.time() >= m_time && m_state==40){
				this->stopRobotMove();    // at first, stop robot maneuver

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + evt.time();
				m_state = 50;
			}
			break;
		}
		case 50: {  // detour: rotate toward relay point 1
			if(evt.time() >= m_time && m_state==50){
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_relayPoint1);

				m_time = l_moveTime+evt.time();
				m_state = 60;
			}
			break;
		}
		case 60: {  // detour: go toward relay point 1
			if(evt.time() >= m_time && m_state==60){
				this->stopRobotMove();

				double l_moveTime = goToObj(m_relayPoint1, 0.0);

				m_time = l_moveTime+evt.time();
				m_state = 70;
			}
			break;
		}
		case 70: {  // rotate toward the position in front of trash
			if(evt.time() >= m_time && m_state==70){
				this->stopRobotMove();

				double l_moveTime = rotateTowardObj(m_frontTrash1);

				m_time = l_moveTime+evt.time();
				m_state = 80;
			}
			break;
		}
		case 80: {  // go toward the position in front of trash
			if(evt.time() >= m_time && m_state==80){
				this->stopRobotMove();

				double l_moveTime = goToObj(m_frontTrash1, 0.0);

				m_time = l_moveTime+evt.time();
				m_state = 90;
			}
			break;
		}
		case 90: {  // rotate toward the trash
			if(evt.time() >= m_time && m_state==90){
				this->stopRobotMove();

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime+evt.time();
				m_state = 100;
			}
			break;
		}
		case 100: {  // prepare the robot arm to grasping the trash
			if(evt.time() >= m_time && m_state==100){
				this->stopRobotMove();
				this->neutralizeArms(evt.time());

				m_state = 105;
			}
			break;
		}
		case 105: {  // fix robot direction for grasping
			if(evt.time() >= m_time1 && m_state==105) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==105) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==105){
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = rotateTowardObj(l_tpos);

				m_time = l_moveTime+evt.time();

				m_state = 110;
			}
			break;
		}
		case 110: {  // approach to the trash
			if(evt.time() >= m_time && m_state==110){
				this->stopRobotMove();

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = goToObj(l_tpos, 30.0);
				m_time = l_moveTime+evt.time();

				m_state = 120;
			}
			break;
		}
		case 120: {  // try to grasp trash
			if(evt.time() >= m_time && m_state==120){
				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName2);
				double l_moveTime = goGraspingObject(l_tpos);
				m_time = l_moveTime+evt.time();

				m_state = 125;
			}
			break;
		}
		case 125: {
			if(evt.time() >= m_time && m_state==125) {
				m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				this->neutralizeArms(evt.time());

				m_state = 130;
			}
			break;
		}
		case 130: {
			if(evt.time() >= m_time1 && m_state==130) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==130) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==130){

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + evt.time();

				m_state = 150;
			}
			break;
		}
		case 150: {
			if(evt.time() >= m_time && m_state==150){
				this->stopRobotMove();
				double l_moveTime = rotateTowardObj(m_frontTrashBox2);

				m_time = l_moveTime + evt.time();
				m_state = 160;
			}
			break;
		}
		case 160: {
			if(evt.time() >= m_time && m_state==160){
				this->stopRobotMove();
				double l_moveTime = goToObj(m_frontTrashBox2,0.0);
				m_time = l_moveTime + evt.time();
				m_state = 161;
			}
			break;
		}
		case 161: {
			if(evt.time() >= m_time && m_state==161){
				this->stopRobotMove();
				this->prepareThrowing(evt.time());

				m_state = 165;
			}
			break;
		}
		case 165: {
			if(evt.time() >= m_time1 && m_state==165) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==165) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==165){

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 170;
			}
			break;
		}
		case 170: {
			if(evt.time() >= m_time && m_state==170){

				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = goToObj(l_tpos, 50.0);
				m_time = l_moveTime + evt.time();

				m_state = 180;
			}
			break;
		}
		case 180: {
			if(evt.time() >= m_time && m_state==180){
				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName2);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 200;
			}
			break;
		}
		case 200: {  // throw trash and get back a bit
			if(evt.time() >= m_time && m_state==200){
				this->stopRobotMove();
				this->throwTrash();

				sleep(1);

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 50.0/m_movingSpeed + evt.time();

				m_state = 225;
			}
			break;
		}
		case 225: {  // recover robot arms
			if(evt.time() >= m_time && m_state==225){
				this->stopRobotMove();
				this->neutralizeArms(evt.time());

				m_state = 240;
			}
			break;
		}
//********************************************************************
		case 240: {  // go next
			if(evt.time() >= m_time1 && m_state==240) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==240) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==240){
				this->stopRobotMove();

				m_graspObjectName = m_trashName1;  // set next target

				double l_moveTime = rotateTowardObj(m_frontTrash2);
				m_time = l_moveTime + evt.time();

				m_state = 250;
			}
			break;
		}
		case 250: {  // approach to neighbor of next target
			if(evt.time() >= m_time && m_state==250){
				this->stopRobotMove();

				double l_moveTime = goToObj(m_frontTrash2, 0.0);
				m_time = l_moveTime + evt.time();

				m_state = 260;
			}
			break;
		}
		case 260: {
			if(evt.time() >= m_time && m_state==260){
				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime+evt.time();

				m_state = 270;
			}
			break;
		}
		case 270: {  // approach to next target
			if(evt.time() >= m_time && m_state==270){
				this->stopRobotMove();

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = goToObj(l_tpos, 39.0);
				m_time = l_moveTime + evt.time();

				m_state = 275;
			}
			break;
		}
		case 275: {
			if(evt.time() >= m_time && m_state==275){
				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime+evt.time();

				m_state = 280;
			}
			break;
		}
		case 280: {
			if(evt.time() >= m_time && m_state==280){
				this->stopRobotMove();

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashName1);
				double l_moveTime = goGraspingObject(l_tpos);
				m_time = l_moveTime+evt.time();

				m_state = 290;
			}
			break;
		}
		case 290: {
			if(evt.time() >= m_time && m_state==290) {
				m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				this->neutralizeArms(evt.time());

				m_state = 300;
			}
			break;
		}
		case 300: {
			if(evt.time() >= m_time1 && m_state==300) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==300) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==300){

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 20./m_movingSpeed + evt.time();

				m_state = 310;
			}
			break;
		}
		case 310: {
			if(evt.time() >= m_time && m_state==310){
				this->stopRobotMove();
				double l_moveTime = rotateTowardObj(m_frontTrashBox1);

				m_time = l_moveTime + evt.time();
				m_state = 320;
			}
			break;
		}
		case 320: {
			if(evt.time() >= m_time && m_state==320){
				this->stopRobotMove();
				double l_moveTime = goToObj(m_frontTrashBox1,0.0);
				m_time = l_moveTime + evt.time();

				m_state = 340;
			}
			break;
		}
		case 340: {
			if(evt.time() >= m_time && m_state==340){
				this->stopRobotMove();
				this->prepareThrowing(evt.time());

				m_state = 350;
			}
			break;
		}
		case 350: {
			if(evt.time() >= m_time1 && m_state==350) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
			if(evt.time() >= m_time4 && m_state==350) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
			if(evt.time() >= m_time1 && evt.time() >= m_time4 && m_state==350){

				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 360;
			}
			break;
		}
		case 360: {
			if(evt.time() >= m_time && m_state==360){

				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				double l_moveTime = goToObj(l_tpos, 50.0);
				m_time = l_moveTime + evt.time();

				m_state = 370;
			}
			break;
		}
		case 370: {
			if(evt.time() >= m_time && m_state==370){
				this->stopRobotMove();
				Vector3d l_tpos;
				this->recognizeObjectPosition(l_tpos, m_trashBoxName1);
				double l_moveTime = rotateTowardObj(l_tpos);
				m_time = l_moveTime + evt.time();

				m_state = 380;
			}
			break;
		}
		case 380: {  // throw trash and get back a bit
			if(evt.time() >= m_time && m_state==380){
				this->stopRobotMove();
				this->throwTrash();

				sleep(1);

				m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
				m_time = 50.0/m_movingSpeed + evt.time();

				m_state = 390;
			}
			break;
		}
		case 390: {  // recover robot arms
			if(evt.time() >= m_time && m_state==390){
				this->stopRobotMove();

				m_state = 0;
			}
			break;
		}
	}

	return refreshRateOnAction;
}


void DemoRobotController::onRecvMsg(RecvMsgEvent &evt) {
}


void DemoRobotController::onCollision(CollisionEvent &evt) {
	if (m_grasp == false){
		typedef CollisionEvent::WithC C;
		//触れたエンティティの名前を得ます
		const std::vector<std::string> & with = evt.getWith();
		// 衝突した自分のパーツを得ます  
		const std::vector<std::string> & mparts = evt.getMyParts();

		//　衝突したエンティティでループします
		for(int i = 0; i < with.size(); i++){
			if(m_graspObjectName == with[i]){
				//右手に衝突した場合
				if(mparts[i] == "RARM_LINK7"){
					//自分を取得
					SimObj *my = getObj(myname());
					//自分の手のパーツを得ます
					CParts * parts = my->getParts("RARM_LINK7");
					if(parts->graspObj(with[i])) m_grasp = true;
				}
			}
		}
	}
}


void DemoRobotController::stopRobotMove(void) {
	m_robotObject->setWheelVelocity(0.0, 0.0);
}


/*
void DemoRobotController::stopRobotArmMove(void) {
	m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
}
*/


double DemoRobotController::goToObj(Vector3d pos, double range) {
	// get own position
	Vector3d robotCurrentPosition;
	//m_robotObject->getPosition(robotCurrentPosition);
	m_robotObject->getPartsPosition(robotCurrentPosition,"RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= robotCurrentPosition;

	// ignore y-direction
	l_pos.y(0);

	// measure actual distance
	double distance = l_pos.length() - range;

	// start moving
	m_robotObject->setWheelVelocity(m_angularVelocity, m_angularVelocity);

	// time to be elapsed
	double l_time = distance / m_movingSpeed;

	return l_time;
}


double DemoRobotController::rotateTowardObj(Vector3d pos) {  // "pos" means target position
	// get own position
	Vector3d ownPosition;
	m_robotObject->getPartsPosition(ownPosition,"RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= ownPosition;

	// ignore variation on y-axis
	l_pos.y(0);

	// get own rotation matrix
	Rotation ownRotation;
	m_robotObject->getRotation(ownRotation);

	// get angles arround y-axis
	double qw = ownRotation.qw();
	double qy = ownRotation.qy();
	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0) theta = -1.0*theta;

	// rotation angle from z-axis to x-axis
	double tmp = l_pos.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(l_pos.x() > 0) targetAngle = -1.0*targetAngle;
	targetAngle += theta;

	double angVelFac = 3.0;
	double l_angvel = m_angularVelocity/angVelFac;

	if(targetAngle == 0.0){
		return 0.0;
	}
	else {
		// 回転すべき円周距離
		double l_distance = m_distance*M_PI*fabs(targetAngle)/(2.0*M_PI);

		// 回転時間(u秒)
		double l_time = l_distance / (m_movingSpeed/angVelFac);

		// 車輪回転開始
		if(targetAngle > 0.0){
			m_robotObject->setWheelVelocity(l_angvel, -l_angvel);
		}
		else{
			m_robotObject->setWheelVelocity(-l_angvel, l_angvel);
		}

		return l_time;
	}
}

void DemoRobotController::recognizeObjectPosition(Vector3d &pos, std::string &name){
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
}


void DemoRobotController::prepareThrowing(double evt_time){
	double thetaJoint1 = 50.0;
	m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;

	double thetaJoint4 = 65.0;
	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;

}


void DemoRobotController::throwTrash(void){
	// get the part info. 
	CParts *parts = m_robotObject->getParts("RARM_LINK7");

	// release grasping
	parts->releaseObj();

	// wait a bit
	sleep(1);

	// set the grasping flag to neutral
	m_grasp = false;
}


double DemoRobotController::goGraspingObject(Vector3d &pos){
	double l_time;
	double thetaJoint4 = 20.0;

	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

	return l_time;
}


void DemoRobotController::neutralizeArms(double evt_time){
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0/(M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
	double thetaJoint1 = -15 - angleJoint1;
	double thetaJoint4 = -110 - angleJoint4;

	if(thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	if(thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;
}


//********************************************************************
extern "C" Controller * createController() {  
  return new DemoRobotController;  
}  
