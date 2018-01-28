#pragma once
#include <deque>
#include "RbtInterfStatus.h"
#include "Seq.h"
#include "SerialRobotModel.h"


class Interpolation
{
public:
	Interpolation(std::shared_ptr<SerialRobotModel> pR) : pRobot(pR)
	{
	};
	~Interpolation()
	{
	};

	/**
	* @class tagQuinticCoe Interpolation.h
	* @brief 五次多项式系数
	*/
	typedef struct tagQuinticCoe
	{
		double a0; /**< @brief 系数0 */
		double a1; /**< @brief 系数1 */
		double a2; /**< @brief 系数2 */
		double a3; /**< @brief 系数3 */
		double a4; /**< @brief 系数4 */
		double a5; /**< @brief 系数5 */
	} QuinticCoe; /**< @brief 五次多项式的系数 */

	typedef struct tagTrapezoidalCoe
	{
		double a0; /**< @brief 系数0 */
		double a1; /**< @brief 系数1 */
		double a2; /**< @brief 系数2 */
	} trapezoidalCoe; /**< @brief二次多项式的系数 */

	void getQuinticCoe(double q0, double qf, double dq0, double dqf,
		double ddq0, double ddqf, double tf, QuinticCoe &Coe);
	void getTrapezoidalCoe(double q0, double qf, double dq0, double dqf,
		double tf, trapezoidalCoe &Coe);
	void trapezoidalPolynomi(double r, trapezoidalCoe& coe, double *qptr);
	void QuinticPolynomi(double r, QuinticCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);

	/**< @brief 五次多项式 */
	/**
	* @class tagQuarticCoe JogInterp.h
	* @brief 四次多项式系数
	*/
	typedef struct tagQuarticCoe
	{
		double a0; /**< @brief 系数0 */
		double a1; /**< @brief 系数1 */
		double a2; /**< @brief 系数2 */
		double a3; /**< @brief 系数3 */
		double a4; /**< @brief 系数4 */
	} QuarticCoe; /**< @brief 四次多项式的系数 */
	void getQuarticCoe1(double q0, double dq0, double ddq0, double ddqh,
		double ddqf, double tf, QuarticCoe &Coe);
	void getQuarticCoe2(double q0, double dq0, double dqf, double ddq0,
		double ddqf, double tf, QuarticCoe &Coe);
	void getQuarticCoe3(double q0, double qf, double dq0, double ddq0,
		double ddqf, double tf, QuarticCoe &Coe);
	void QuarticPolynomi(double r, QuarticCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);

	/**
	* @class tagLinearCoe JogInterp.h
	* @brief 一次多项式系数
	*/
	typedef struct tagLinearCoe
	{
		double a0; /**< @brief 系数0 */
		double a1; /**< @brief 系数1 */
	} LinearCoe; /**< @brief 一次多项式的系数 */

	void LinearPolynomi(double r, LinearCoe& coe, double *qptr,
		double *dqptr = NULL, double *ddqptr = NULL, double *dddqptr = NULL);/**< @brief 一次多项式 */
	void getLinearCoe(double q0, double dq0, double tf, LinearCoe &Coe);


	//计算无线性段的插补点
	void InterpNoLinear_5_0_5(double q0, double qf, double Ts, double tf, int polynomiType,
						   	  std::deque<double> &posInterp, double *posEdptr = NULL,
							  double *velEdptr = NULL, double *accEdptr = NULL,
							  double *jerkEdptr = NULL);
	//计算5阶拟合的线段，尽管这种方法有bug的，应该4阶就够了，先用着
	ErrorID calcInterp_5_1_5(double q0, double q1, double Ts, double maxVel,
							double maxAcc, double maxDecel, double maxJerk, int polynomiType,
							std::deque<double> &seq);

public:
	//关节插补
	ErrorID JointInterp(const Position_ACS_deg &targetPoint,
						const Position_ACS_deg &originPoint, double Ts, double velPerc,
						double accPerc, int polynomiType, N_AxisSeqPtr &nAglSeqPtr); //输入的是弧度值，内部处理使用的是角度值

public:
	//直线插补
	ErrorID LineInterp(const Position_MCS_rad &targetPoint,
		const Position_MCS_rad &originPoint,
		const Position_ACS_rad &originACS, double Ts, double maxVel,
		double maxAcc, double maxDecel, double maxJerk, double maxVelOri,
		double maxAccOri, double maxDecelOri, double maxJerkOri, int polynomiType,
		N_AxisSeqPtr &nAglSeqPtr, GrpTcpSeq *pGrpTcp = NULL);

public:
	//圆弧插补
	ErrorID ArcInterp(const Position_MCS_rad &middlePoint,
		const Position_MCS_rad &targetPoint,
		const Position_MCS_rad &originPoint,
		const Position_ACS_rad &originACS, double Ts, double maxVel,
		double maxAcc, double maxDecel, double maxJerk, double maxVelOri,
		double maxAccOri, double maxDecelOri, double maxJerkOri, int polynomiType, bool isFullCircle,
		N_AxisSeqPtr &nAglSeqPtr, GrpTcpSeq *pGrpTcp = NULL);
private:
	//计算圆心	
	XYZ calcCircleCenter(const XYZ& pA, const XYZ& pB, const XYZ& pC);
	//计算单位法向量
	Vector3 calcOrthoNormalVector(const Vector3& v1, const Vector3& v2);
private:
	std::shared_ptr<SerialRobotModel> pRobot;
};

