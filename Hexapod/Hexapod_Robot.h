#ifndef ROBOT_H
#define ROBOT_H

#include "Aris_DynKer.h"
#include <fstream>



//需要修改默认的堆栈大小
//在windows里，位于程序属性->链接器->系统->堆栈保留大小

/** \brief 命名空间：六足机器人
*
* 用来计算机器人的运动学和动力学等。
*/
namespace Hexapod_Robot
{
	class LEG;
	class ROBOT;

	/** \brief 机器人单腿类型 
	* 
	* 每条腿包含6个部件，9个运动副，3个驱动，以及很多个坐标系。在单腿里使用指针来指向这些变量。
	*
	*/
	class LEG: public Aris::DynKer::OBJECT
	{
		friend class ROBOT;

	public:
		union
		{
			struct
			{
				Aris::DynKer::PART* pP1a;/*!< \brief 指向部件P1a的指针 */
				Aris::DynKer::PART* pP2a;/*!< \brief 指向部件P2a的指针 */
				Aris::DynKer::PART* pP3a;/*!< \brief 指向部件P3a的指针 */
				Aris::DynKer::PART* pThigh;/*!< \brief 指向部件Thigh的指针 */
				Aris::DynKer::PART* pP2b;/*!< \brief 指向部件P2b的指针 */
				Aris::DynKer::PART* pP3b;/*!< \brief 指向部件P3b的指针 */
			};
			Aris::DynKer::PART* pPrts[6];
		};
		union
		{
			struct
			{
				Aris::DynKer::JOINT *pU1;/*!< \brief 指向关节U1的指针 */
				Aris::DynKer::JOINT *pU2;/*!< \brief 指向关节U2的指针 */
				Aris::DynKer::JOINT *pU3;/*!< \brief 指向关节U3的指针 */
				Aris::DynKer::JOINT *pP1;/*!< \brief 指向关节P1的指针 */
				Aris::DynKer::JOINT *pP2;/*!< \brief 指向关节P2的指针 */
				Aris::DynKer::JOINT *pP3;/*!< \brief 指向关节P3的指针 */
				Aris::DynKer::JOINT *pS2;/*!< \brief 指向关节S2的指针 */
				Aris::DynKer::JOINT *pS3;/*!< \brief 指向关节S3的指针 */
				Aris::DynKer::JOINT *pSf;/*!< \brief 指向关节Sf的指针 */
			};
			Aris::DynKer::JOINT *pJnts[9];
		};
		union
		{
			struct
			{
				Aris::DynKer::MARKER *pBase;/*!< \brief 指向坐标系Base的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU1i;/*!< \brief 指向坐标系U1i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU1j;/*!< \brief 指向坐标系U1j的指针，位于部件P1a上 */
				Aris::DynKer::MARKER *pU2i;/*!< \brief 指向坐标系U2i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU2j;/*!< \brief 指向坐标系U2j的指针，位于部件P2a上 */
				Aris::DynKer::MARKER *pU3i;/*!< \brief 指向坐标系U3i的指针，位于部件MainBody上 */
				Aris::DynKer::MARKER *pU3j;/*!< \brief 指向坐标系U3j的指针，位于部件P3a上 */
				Aris::DynKer::MARKER *pP1i;/*!< \brief 指向坐标系P1i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pP1j;/*!< \brief 指向坐标系P1j的指针，位于部件P1a上 */
				Aris::DynKer::MARKER *pP2i;/*!< \brief 指向坐标系P2i的指针，位于部件P2b上 */
				Aris::DynKer::MARKER *pP2j;/*!< \brief 指向坐标系P2j的指针，位于部件P2a上 */
				Aris::DynKer::MARKER *pP3i;/*!< \brief 指向坐标系P3i的指针，位于部件P3b上 */
				Aris::DynKer::MARKER *pP3j;/*!< \brief 指向坐标系P3i的指针，位于部件P3a上 */
				Aris::DynKer::MARKER *pS2i;/*!< \brief 指向坐标系S2i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pS2j;/*!< \brief 指向坐标系S2j的指针，位于部件P2b上 */
				Aris::DynKer::MARKER *pS3i;/*!< \brief 指向坐标系S3i的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pS3j;/*!< \brief 指向坐标系S3j的指针，位于部件P3b上 */
				Aris::DynKer::MARKER *pSfi;/*!< \brief 指向坐标系Sfi的指针，位于部件Thigh上 */
				Aris::DynKer::MARKER *pSfj;/*!< \brief 指向坐标系Sfj的指针，位于部件Ground上 */
			};
			Aris::DynKer::MARKER *pMaks[19];
		};
		union
		{
			struct
			{
				Aris::DynKer::MOTION *pM1;/*!< \brief 指向驱动M1的指针 */
				Aris::DynKer::MOTION *pM2;/*!< \brief 指向驱动M2的指针 */
				Aris::DynKer::MOTION *pM3;/*!< \brief 指向驱动M3的指针 */
			};

			Aris::DynKer::MOTION *pMots[3];
		};

	private:
		ROBOT *pRobot;

		const double U2x,U2y,U2z,U3x,U3y,U3z;
		const double S2x,S2y,S2z,S3x,S3y,S3z;
		const double Sfx,Sfy,Sfz;

		const double D1, H1, D2, H2;
	
		union
		{
			struct
			{
				double x,y,z;
			};
			double pEE[3];
		};
		union
		{
			struct
			{
				double a1, b1;
				union
				{
					struct{ double l1, l2, l3; };
					double pIn[3];
				};
			};
			double pCD[3];
		};
		union
		{
			struct
			{
				double vx, vy, vz;
			};
			double vEE[3];
		};
		union
		{
			struct
			{
				double va1, vb1;
				union
				{
					struct{ double vl1, vl2, vl3; };
					double vIn[3];
				};
			};
			double vCD[3];
		};
		union
		{
			struct
			{
				double ax, ay, az;
			};
			double aEE[3];
		};
		union
		{
			struct
			{
				double aa1, ab1;
				union
				{
					struct{ double al1, al2, al3; };
					double aIn[3];
				};
			};
			double aCD[3];
		};


		double x2, y2, z2, x3, y3, z3;
		double a2, b2, a3, b3;
		double sa1, ca1, sb1, cb1, sa2, ca2, sb2, cb2, sa3, ca3, sb3, cb3;
		double pa1, qa1, pb1, qb1, pa2, qa2, pb2, qb2, pa3, qa3, pb3, qb3;

		double vx2, vy2, vz2, vx3, vy3, vz3;
		double va2, vb2, va3, vb3;

		double ax2, ay2, az2, ax3, ay3, az3;
		double aa2, ab2, aa3, ab3;

		double H11, H12, H21, H22;
		double k21, k22, k23, k31, k32, k33;
		double vk21, vk22, vk23, vk31, vk32, vk33;
		double J1[3][3], J2[3][3], vJ1[3][3], vJ2[3][3];

		double _C[36][36];
		double _I[36][36];
		double _f[36];
		double _v[36];
		double _a_c[36];

		double _a[36];
		double _epsilon[36];

		double _c_M[36][4];

	private:
		/*!
		* \brief 构造函数
		* \param Name 单腿的名字，长度最大为39个字符。
		* \param pRobot 指向所属机器人的指针。
		* \param ep 表示单腿相对于机器人的欧拉角与位置。
		*/
		LEG(const char *Name, ROBOT* pRobot);
		/*!
		* \brief 析构函数
		*/
		~LEG();

	public:
		/*!
		* \brief 获取某个坐标系下的单腿末端位置。
		* \param pEE 地面坐标系中的末端位置，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿主支链的关节坐标。
		* \param pCD 单腿主支链的关节坐标。
		*/
		void GetPcd(double *pCD) const;
		/*!
		* \brief 获取单腿三个输入的位置。
		* \param pIn 单腿三个输入的位置。
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief 获取某个坐标系下的单腿末端速度。
		* \param vEE 地面坐标系中的末端速度，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void GetVee(double *vEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿主支链的关节速度。
		* \param vCD 单腿主支链的关节速度。
		*/
		void GetVcd(double *vCD) const;
		/*!
		* \brief 获取单腿三个输入的速度。
		* \param vIn 单腿三个输入的速度。
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief 获取某个坐标系下的单腿末端加速度。
		* \param aEE 地面坐标系中的末端加速度，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void GetAee(double *aEE, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿主支链的关节加速度。
		* \param aCD 单腿主支链的关节加速度。
		*/
		void GetAcd(double *aCD) const;
		/*!
		* \brief 获取单腿三个输入的加速度。
		* \param aIn 单腿三个输入的加速度。
		*/
		void GetAin(double *aIn) const;

		/*!
		* \brief 获取单腿坐标系下的正向力雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetFceJacDir(double *jac, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向力雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetFceJacInv(double *jac, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的正向速度雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetVelJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向速度雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		*/
		void GetVelJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的正向加速度雅克比矩阵（已知输入求输出）。
		* \param jac 3乘3的雅克比矩阵。
		* \param c 残余项。
		* \param RelativeCoodinate 输出加速度所在的坐标系。
		* 输入输出关系可以用下式计算：
		* Aee=jac*Ain+c
		*/
		void GetAccJacDir(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		/*!
		* \brief 获取单腿坐标系下的拟向加速度雅克比矩阵（已知输出求输入）。
		* \param jac 3乘3的雅克比矩阵。
		* \param c 残余项。
		* \param RelativeCoodinate 输出加速度所在的坐标系。
		* 输入输出关系可以用下式计算：
		* Ain=jac*Aee+c
		*/
		void GetAccJacInv(double *jac, double *c = 0, const char *RelativeCoodinate = "G") const;
		
		/*!
		* \brief 设置某个坐标系下的单腿末端位置。
		* \param pEE 地面坐标系中的末端位置，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void SetPee(const double *pEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief 设置单腿主支链的关节坐标。
		* \param pCD 单腿主支链的关节坐标。
		*/
		void SetPcd(const double *pCD);
		/*!
		* \brief 设置单腿三个输入的位置，在计算正解的时候使用解析方法。
		* \param pIn 单腿三个输入的位置。
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief 设置单腿三个输入的位置，在计算正解的时候使用数值方法。
		* \param pIn 单腿三个输入的位置。
		*/
		void SetPin2(const double *pIn);
		/*!
		* \brief 设置某个坐标系下的单腿末端速度。
		* \param vEE 地面坐标系中的末端速度，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void SetVee(const double *vEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief 设置单腿主支链的关节速度。
		* \param vCD 单腿主支链的关节速度。
		*/
		void SetVcd(const double *vCD);
		/*!
		* \brief 设置单腿三个输入的速度。
		* \param vIn 单腿三个输入的速度。
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief 设置某个坐标系下的单腿末端加速度。
		* \param vEE 地面坐标系中的末端加速度，为3维数组。末端为Sf副。
		* \param RelativeCoodinate pEE的坐标系，"G"、"O"表示地面坐标系，"M"、"B"表示机身坐标系，"L"表示单腿坐标系。
		*/
		void SetAee(const double *aEE, const char *RelativeCoodinate = "G");
		/*!
		* \brief 设置单腿主支链的关节加速度。
		* \param vCD 单腿主支链的关节加速度。
		*/
		void SetAcd(const double *aCD);
		/*!
		* \brief 设置单腿三个输入的加速度。
		* \param vIn 单腿三个输入的加速度。
		*/
		void SetAin(const double *aIn);

	public:
		void FastDynMtxInPrt();
		void FastDynEeForce(const double *fIn_in, double *fEE_out, const char *RelativeCoodinate = "G");

	public:
		void Calibrate(const int n, const double *plen, const double *vplen, const double *aplen, const double *fplen);

	private:
		void _CalCdByPos();
		void _CalCdByPlen();
		void _CalCdByPlen2();
		void _CalVarByCd();
		void _CalPartByVar();

		void _CalVcdByVpos();
		void _CalVcdByVplen();
		void _CalVvarByVcd();
		void _CalVpartByVvar();

		void _CalAcdByApos();
		void _CalAcdByAplen();
		void _CalAvarByAcd();
		void _CalApartByAvar();
	};
	/** \brief 机器人类型
	*
	* 用于计算整个机器人的运动学，包含依次指向每条腿的指针。
	*
	*/
	class ROBOT:public Aris::DynKer::MODEL
	{
		friend class LEG;

	public:
		Aris::DynKer::PART* pBody;/*!< \brief 指向部件Body的指针 */
		Aris::DynKer::MARKER* pBodyCenter;/*!< \brief 指向机器人机身中心坐标系 */

	private:
		LEG LF_Leg, LM_Leg, LR_Leg, RF_Leg, RM_Leg, RR_Leg;

	public:
		union
		{
			struct
			{
				LEG *pLF;/*!< \brief 指向机器人左前腿的指针 */
				LEG *pLM;/*!< \brief 指向机器人左中腿的指针 */
				LEG *pLR;/*!< \brief 指向机器人左后腿的指针 */
				LEG *pRF;/*!< \brief 指向机器人右前腿的指针 */
				LEG *pRM;/*!< \brief 指向机器人右中腿的指针 */
				LEG *pRR;/*!< \brief 指向机器人右后腿的指针 */
			};
			LEG *pLegs[6];
		};

		double result[18];

	public:
		ROBOT();/*!< \brief 构造函数 */
		~ROBOT();/*!< \brief 析构函数 */

		int LoadXML(const char *filename);

		void GetPee(double *pEE, const char *RelativeCoodinate = "G") const;/*!< \brief 等同于GetPeeG函数 */
		void GetPin(double *pIn) const;/*!< \brief 获取6条腿的丝杠位置 */
		void GetBodyPm(double *bodypm) const;/*!< \brief 获取身体位置 */
		void GetVee(double *vEE, const char *RelativeCoodinate = "G") const;/*!< \brief 等同于GetVeesG函数 */
		void GetVin(double *vIn) const;/*!< \brief 获取6条腿的丝杠速度 */
		void GetBodyVel(double *bodyvel) const;/*!< \brief 获取身体速度 */
		void GetAee(double *aEE, const char *RelativeCoodinate = "G") const;/*!< \brief 等同于GetVeesG函数 */
		void GetAin(double *aIn) const;/*!< \brief 获取6条腿的丝杠速度 */
		void GetBodyAcc(double *bodyacc) const;/*!< \brief 获取身体速度 */
		void SetPee(const double *pEE = 0, const double *bodyep = 0, const char *RelativeCoodinate = "G");/*!< \brief 等同于SetPeesG函数 */
		void SetPin(const double *pIn = 0, const double *bodyep = 0);/*!< \brief 设置6条腿的丝杠位置 */
		void SetVee(const double *vEE = 0, const double *bodyvel = 0, const char *RelativeCoodinate = "G");/*!< \brief 等同于SetVeeG函数 */
		void SetVin(const double *vIn = 0, const double *bodyvel = 0);/*!< \brief 设置6条腿的丝杠速度 */
		void SetAee(const double *aEE = 0, const double *bodyacc = 0, const char *RelativeCoodinate = "G");/*!< \brief 等同于SetVeeG函数 */
		void SetAin(const double *aIn = 0, const double *bodyacc = 0);/*!< \brief 设置6条腿的丝杠速度 */

		void GetVelJacDir(double *jac, const char* = 0) const;/*!< \brief 获取G坐标系下的正雅克比矩阵，从丝杠输入到身体位姿 */
		void GetVelJacInv(double *jac, const char* = 0) const;/*!< \brief 获取G坐标系下的逆雅克比矩阵，从身体位姿到丝杠输入 */

		void SetFixedFeet(const char *fixedLeg = 0, const char *ActiveMotion = 0);
		void SetVinWithFixedFeet(const double *vIn = 0);
		void SetAinWithFixedFeet(const double *aIn = 0);

		void FastDynMtxInPrt();
		void FastDynEeForce(const double *fIn_in, double *fEE_out, const char *RelativeCoodinate = "G");
		void Clb(const char* filename);
		void ClbLeg(const char* filename);
		void TestClb();
		void TestClbEeFce();

		int MoveWithKinect(double* currentH, double *nextH, double *data);

		virtual void SaveAdams(const char *filename)
		{
			MODEL::SaveAdams(filename);

			//ofstream file;

			//file.open(filename);

		}
	};
}

#endif