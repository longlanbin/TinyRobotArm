#pragma once
#include "RobotECM.h"
#include "Seq.h"
#include "General6S.h"
#include "Scara4S.h"
#include "Interpolation.h"

class Robot
{
public:
	Robot(std::shared_ptr<SerialRobotModel> pR);
	~Robot();
	static Robot* getNewRobot();
	void setRobotParam();
	void encoderReset(unsigned int servoNum);
	void servoOff();
	void servoOn();
	int targetPosAssign(int& Pos, int axisIndex, N_AxisSeqPtr& SeqPtr);
	int targetPosAssignGroup(double Pos[], N_AxisSeqPtr& SeqPtr);

	inline int degToPul(double var, int encoder) const { return ((signed int)(var*(1 << encoder) / 360.0)); };

	void printCurPos();
	void readNowPos(Position &pos);
	int mBt_Ptp_Single();
	int mBt_Ptp_Group(COORD_SYS coord);
	int mBt_MoveLine_Group();
	int mBt_MoveArc_Group();
	//int mBt_Jog(int index);

	static USER_DAT* pData;     //指向共享内存的指针
	bool robotRunSingle;
	bool robotRunGroup;
	int encoderResolution[TOTAL_AXES];

private:
	inline double pulToDeg(int var, int encoder) const { return (((double)var) * 360 / (1 << encoder)); };
	void getReducRatio();
	void getDirection();
	void getEncoderResolution();
	void getCoupleCoe();

	int robotType;
	double Ts; /**< @brief 控制周期，设置为1ms*/
	size_t totalAxes;

	int direction[TOTAL_AXES];
	double reducRatio[TOTAL_AXES];
	AXIS_Couple_Coe coupleCoe;
	double singleTurnEncoder[TOTAL_AXES];
	std::shared_ptr<SerialRobotModel> pRobotModel;
	std::shared_ptr<Interpolation> pInterp;
	std::vector<std::shared_ptr<SerialRobotModel> > pRobotModelVec;
};

