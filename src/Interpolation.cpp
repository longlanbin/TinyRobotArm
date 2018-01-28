#include "Interpolation.h"
#include <iostream>


//梯型加速,求系数
void Interpolation::getTrapezoidalCoe(double q0, double qf, double dq0, double dqf,
	double tf, trapezoidalCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = (dqf - dq0) / 2.0 / tf;
}

//二次多项式求解
void Interpolation::trapezoidalPolynomi(double r, trapezoidalCoe& coe, double *qptr)
{
	*qptr = coe.a0 + coe.a1*r + coe.a2*pow(r, 2);
}

//求系数a0,a1,a2...a5
void Interpolation::getQuinticCoe(double q0, double qf, double dq0, double dqf,
	double ddq0, double ddqf, double tf, QuinticCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2.0;
	Coe.a3 = (20 * qf - 20 * q0 - (8 * dqf + 12 * dq0)*tf - (3 * ddq0 - ddqf)*tf *tf) / (2 * pow(tf, 3));
	Coe.a4 = (30 * q0 - 30 * qf + (14 * dqf + 16 * dq0)*tf + (3 * ddq0 - 2 * ddqf) *tf*tf) / (2 * pow(tf,
		4));
	Coe.a5 = (12 * qf - 12 * q0 - (6 * dq0 + 6 * dqf)*tf - (ddq0 - ddqf)*tf*tf) / (2 * pow(tf, 5));
}

//五次多项式求解
void Interpolation::QuinticPolynomi(double r, QuinticCoe& coe, double *qptr,
	double *dqptr/*=NULL*/, double *ddqptr/*=NULL*/, double *dddqptr/*=NULL*/)
{
	if (qptr)
		*qptr = coe.a0 + coe.a1*r + coe.a2*pow(r, 2) + coe.a3*pow(r, 3) + coe.a4*pow(r,
			4) + coe.a5*pow(r, 5);
	if (dqptr)
		*dqptr = coe.a1 + 2 * coe.a2*r + 3 * coe.a3*pow(r, 2) + 4 * coe.a4*pow(r, 3) + 5
		* coe.a5 *pow(r, 4);
	if (ddqptr)
		*ddqptr = 2 * coe.a2 + 6 * coe.a3*r + 12 * coe.a4*pow(r, 2) + 20 * coe.a5*pow(r, 3);
	if (dddqptr)
		*dddqptr = 6 * coe.a3 + 24 * coe.a4*r + 60 * coe.a5*pow(r, 2);
}

//求系数a0,a1,a2...a4
//起点：位置、速度、加速度；中点：加速度； 终点：加速度。
void Interpolation::getQuarticCoe1(double q0, double dq0, double ddq0,
	double ddqh, double ddqf, double tf, QuarticCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2;
	Coe.a3 = (3 * ddq0 - ddqf + 4 * ddqh) / (6 * tf);
	Coe.a4 = (ddq0 + ddqf - 2 * ddqh) / (6 * pow(tf, 2));
}

//求系数a0,a1,a2...a4
//起点：位置、速度、加速度；终点：速度、加速度。
void Interpolation::getQuarticCoe2(double q0, double dq0, double dqf, double ddq0,
	double ddqf, double tf, QuarticCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2.0;
	Coe.a3 = (3 * dqf - 3 * dq0 - 2 * ddq0*tf - ddqf*tf) / (3 * pow(tf, 2));
	Coe.a4 = (2 * dq0 - 2 * dqf + ddq0*tf + ddqf*tf) / (4 * pow(tf, 3));
}

//求系数a0,a1,a2...a4
//起点：位置、速度、加速度；终点：位置、加速度。
void Interpolation::getQuarticCoe3(double q0, double qf, double dq0, double ddq0,
	double ddqf, double tf, QuarticCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2.0;
	Coe.a3 = (12 * qf - 12 * q0 - 12 * dq0*tf - 5 * ddq0*pow(tf, 2) - ddqf*pow(tf, 2))
		/ (6 * pow(tf, 3));
	Coe.a4 = (6 * q0 - 6 * qf + 6 * dq0*tf + 2 * ddq0*pow(tf, 2) + ddqf*pow(tf, 2)) / (6
		* pow(tf, 4));
}

//四次多项式求解
void Interpolation::QuarticPolynomi(double r, QuarticCoe& coe, double *qptr,
	double *dqptr/*=NULL*/, double *ddqptr/*=NULL*/, double *dddqptr/*=NULL*/)
{
	if (qptr)
		*qptr = coe.a0 + coe.a1*r + coe.a2*pow(r, 2) + coe.a3*pow(r, 3) + coe.a4*pow(r,
			4);
	if (dqptr)
		*dqptr = coe.a1 + 2 * coe.a2*r + 3 * coe.a3*pow(r, 2) + 4 * coe.a4*pow(r, 3);
	if (ddqptr)
		*ddqptr = 2 * coe.a2 + 6 * coe.a3*r + 12 * coe.a4*pow(r, 2);
	if (dddqptr)
		*dddqptr = 6 * coe.a3 + 24 * coe.a4*r;
}

void Interpolation::getLinearCoe(double q0, double dq0, double tf, LinearCoe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
}

//一次多项式求解
void Interpolation::LinearPolynomi(double r, LinearCoe& coe, double *qptr,
	double *dqptr/*=NULL*/, double *ddqptr/*=NULL*/, double *dddqptr/*=NULL*/)
{
	if (qptr)
		*qptr = coe.a0 + coe.a1*r;
	if (dqptr)
		*dqptr = coe.a1;
	if (ddqptr)
		*ddqptr = 0;
	if (dddqptr)
		*dddqptr = 0;
}

//只保留位置信息
void Interpolation::InterpNoLinear_5_0_5(double q0, double qf, double Ts, double tf,
	int polynomiType, std::deque<double> &posInterp, double *posEdptr /*=NULL*/,
	double *velEdptr /*=NULL*/, double *accEdptr /*=NULL*/,
	double *jerkEdptr /*=NULL*/)
{
	double t1 = tf / 2;
	size_t N1 = size_t(t1 / Ts);//加速段插补点数
	double t2 = tf - N1*Ts;
	size_t N2 = size_t(t2 / Ts) + 1;//减速段插补点个数

	std::cout << "\n InterpNoLinear_5_0_5: N1=" << N1 << " " << "N2=" << N2 << std::endl;
	std::deque<double> dq, ddq, dddq;
	//位置
	posInterp.resize(N1 + N2, 0);
	//速度
	if (velEdptr)
		dq.resize(N1 + N2, 0);
	else
		dq.resize(N1, 0);
	//加速度
	if (accEdptr)
		ddq.resize(N1 + N2, 0);
	else
		ddq.resize(N1, 0);
	//加加速度
	if (jerkEdptr)
		dddq.resize(N1 + N2, 0);

	//加速段
	QuarticCoe Coe1;
	getQuarticCoe3(q0, qf / 2, 0/*dq0*/, 0/*ddq0*/, 0/*ddqf*/, t1, Coe1);
	for (size_t i = 0; i<N1; ++i)
	{
		QuarticPolynomi((i + 1)*Ts, Coe1, &(posInterp[i]), &(dq[i]), &(ddq[i]),
			(jerkEdptr ? (&(dddq[i])) : 0));
	}
	double posHalf = posInterp[N1 - 1];
	double velHalf = dq[N1 - 1];
	double accHalf = ddq[N1 - 1];

	//减速段
	QuinticCoe Coe2;
	getQuinticCoe(posHalf, qf, velHalf/*dq0*/, 0/*dqf*/, accHalf/*ddq0*/,
		0/*ddqf*/, t2, Coe2);
	for (size_t i = 0; i<N2 - 1; ++i)
	{
		QuinticPolynomi((i + 1)*Ts, Coe2, &(posInterp[i + N1]),
			(velEdptr ? (&(dq[i + N1])) : 0),
			(accEdptr ? (&(ddq[i + N1])) : 0), (jerkEdptr ? (&(dddq[i + N1]))
				: 0));
	}
	QuinticPolynomi(t2, Coe2, &(posInterp[N1 + N2 - 1]),
		(velEdptr ? (&(dq[N1 + N2 - 1])) : 0), (accEdptr ? (&(ddq[N1 + N2 - 1]))
			: 0), (jerkEdptr ? (&(dddq[N1 + N2 - 1])) : 0));

	if (posEdptr)
		*posEdptr = posInterp[N1 + N2 - 1];
	if (velEdptr)
		*velEdptr = dq[N1 + N2 - 1];
	if (accEdptr)
		*accEdptr = ddq[N1 + N2 - 1];
	if (jerkEdptr)
		*jerkEdptr = dddq[N1 + N2 - 1];
}

/*****************************************************************************/
/**
* @brief 5-1-5, min Jerk
*/
/*****************************************************************************/
ErrorID Interpolation::calcInterp_5_1_5(double q0, double q1, double Ts,
	double maxVel, double maxAcc, double maxDecel, double maxJerk, int polynomiType,
	std::deque<double> &seq)
{
	double dis = q1 - q0;
	if (!maxVel)
		return PLCOPEN_MOVE_INSTRUCT_ARG_ERROR;

	//第一步：若全程视作匀速运动，确定最大运动时间T
	double T = fabs(dis / maxVel);
	if (!T)
	{
		return PLCOPEN_NO_MOVE_IS_NEEDED;
	}

	//第二步：确定加（减）速段时间tAcc
	double tAcc = fabs(maxVel / maxAcc);
	double tDecel = fabs(maxVel / maxDecel);

	//第三步：确定匀速段时间tConst
	double tf = tAcc / 2 + T + tDecel / 2; //最终整个过程运行时间为tf
	double tConst = tf - tAcc - tDecel;
	//判断是否有匀速段
	if (tConst<0.75*Ts)
	{
		//只有变速段，没有匀速段处理
		InterpNoLinear_5_0_5(q0, q1, Ts, tAcc + tDecel, polynomiType, seq);//todo：时间没取到让加速度运行到最大值
	}
	else
	{
		//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3）,并确定最终运动时间
		size_t N1 = static_cast<size_t>(tAcc / Ts);
		size_t N2 = static_cast<size_t>(tConst / Ts);
		size_t N3 = static_cast<size_t>(tDecel / Ts);
		std::cout << "\n calcInterp_5_1_5: N1="<<N1 <<" " << "N2=" << N2 << " " << "N3=" << N3 << " " << std::endl;

		tAcc = N1 * Ts;
		tDecel = N3 * Ts;
		tConst = N2 * Ts;

		T = tConst + tAcc / 2 + tDecel / 2;

		//第五步：计算轴最终运行时的平均速度，就是有符号的maxVel
		double velConst = dis / T;//有正负号

								  //第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
		seq.clear();
		double posInterp = 0;//用于暂存轴某周期的插补点位置

		if (polynomiType == 0)
		{
			//(1)加速段插补 [ 第 0 ~ N1 点]
			//计算五次多项式系数
			double accPosEnd;//轴加速段的结束点位置
			QuinticCoe coe;//轴的五次多项式
			accPosEnd = q0 + velConst*tAcc / 2;
			getQuinticCoe(q0, accPosEnd, 0, velConst, 0, 0, tAcc, coe);

			//生成链表
			for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
			{
				QuinticPolynomi(i*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}

			//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
			//生成链表
			for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
			{
				posInterp += velConst*Ts;
				seq.push_back(posInterp);
			}

			//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
			//计算五次多项式系数
			getQuinticCoe(posInterp, q1, velConst, 0, 0, 0, tDecel, coe);

			//生成链表
			for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
			{
				QuinticPolynomi(k*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}
		}
		else if (polynomiType == 1)
		{
			//(1)加速段插补 [ 第 0 ~ N1 点]
			//计算2次多项式系数
			double accPosEnd;//轴加速段的结束点位置
			trapezoidalCoe coe;//轴的2次多项式
			accPosEnd = q0 + velConst*tAcc / 2;
			getTrapezoidalCoe(q0, accPosEnd, 0, velConst, tAcc, coe);

			//生成链表
			for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
			{
				trapezoidalPolynomi(i*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}

			//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
			//生成链表
			for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
			{
				posInterp += velConst*Ts;
				seq.push_back(posInterp);
			}

			//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
			//计算五次多项式系数
			getTrapezoidalCoe(posInterp, q1, velConst, 0, tDecel, coe);

			//生成链表
			for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
			{
				trapezoidalPolynomi(k*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}
		}
	}
	return RBTINTERF_NO_ERROR;
}

//计算圆心
XYZ Interpolation::calcCircleCenter(const XYZ& pA, const XYZ& pB, const XYZ& pC)
{
	XYZ res;
	double x1 = pA(0), y1 = pA(1), z1 = pA(2);
	double x2 = pB(0), y2 = pB(1), z2 = pB(2);
	double x3 = pC(0), y3 = pC(1), z3 = pC(2);
	double a11 = (y2 - y1)*(z3 - z1) - (y3 - y1)*(z2 - z1);
	double a12 = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
	double a13 = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
	double b11 = x1*a11 + y1*a12 + z1*a13;
	double a21 = x2 - x1, a22 = y2 - y1, a23 = z2 - z1;
	double b21 = (pow(x2, 2) + pow(y2, 2) + pow(z2, 2) - pow(x1, 2) - pow(y1, 2) - pow(z1,
		2)) / 2;
	double a31 = x3 - x1, a32 = y3 - y1, a33 = z3 - z1;
	double b31 = (pow(x3, 2) + pow(y3, 2) + pow(z3, 2) - pow(x1, 2) - pow(y1, 2) - pow(z1,
		2)) / 2;

	//dmatrix m(3, 3);
	Matrix3 m(3, 3);
	m << a11, a12, a13, a21, a22, a23, a31, a32, a33;
	XYZ v;
	v << b11, b21, b31;

	res = m.inverse()*v;

	return res;
}

//计算单位正交向量
Vector3 Interpolation::calcOrthoNormalVector(const Vector3& v1, const Vector3& v2)
{
	Vector3 res;
	res = v1.cross(v2);
	res.normalize();//单位化
	return res;
}