#ifndef ROBOT_III_H
#define ROBOT_III_H

#include <Aris_DynModel.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>

namespace Robots
{
	class RobotTypeI;
	class LegI :public Robots::LegBase
	{
	public:
		union
		{
			struct
			{
				Aris::DynKer::Part* pP1a;/*!< \brief 指向部件P1a的指针 */
				Aris::DynKer::Part* pP2a;/*!< \brief 指向部件P2a的指针 */
				Aris::DynKer::Part* pP3a;/*!< \brief 指向部件P3a的指针 */
				Aris::DynKer::Part* pThigh;/*!< \brief 指向部件Thigh的指针 */
				Aris::DynKer::Part* pP2b;/*!< \brief 指向部件P2b的指针 */
				Aris::DynKer::Part* pP3b;/*!< \brief 指向部件P3b的指针 */
			};
			Aris::DynKer::Part* pPrts[6];
		};
		union
		{
			struct
			{
				Aris::DynKer::JointBase *pU1;/*!< \brief 指向关节U1的指针 */
				Aris::DynKer::JointBase *pU2;/*!< \brief 指向关节U2的指针 */
				Aris::DynKer::JointBase *pU3;/*!< \brief 指向关节U3的指针 */
				Aris::DynKer::JointBase *pP1;/*!< \brief 指向关节P1的指针 */
				Aris::DynKer::JointBase *pP2;/*!< \brief 指向关节P2的指针 */
				Aris::DynKer::JointBase *pP3;/*!< \brief 指向关节P3的指针 */
				Aris::DynKer::JointBase *pS2;/*!< \brief 指向关节S2的指针 */
				Aris::DynKer::JointBase *pS3;/*!< \brief 指向关节S3的指针 */
				Aris::DynKer::JointBase *pSf;/*!< \brief 指向关节Sf的指针 */
			};
			Aris::DynKer::JointBase *pJnts[9];
		};
		union
		{
			struct
			{
				Aris::DynKer::Marker *pBase;/*!< \brief 指向坐标系Base的指针，位于部件MainBody上 */
				Aris::DynKer::Marker *pU1i;/*!< \brief 指向坐标系U1i的指针，位于部件MainBody上 */
				Aris::DynKer::Marker *pU1j;/*!< \brief 指向坐标系U1j的指针，位于部件P1a上 */
				Aris::DynKer::Marker *pU2i;/*!< \brief 指向坐标系U2i的指针，位于部件MainBody上 */
				Aris::DynKer::Marker *pU2j;/*!< \brief 指向坐标系U2j的指针，位于部件P2a上 */
				Aris::DynKer::Marker *pU3i;/*!< \brief 指向坐标系U3i的指针，位于部件MainBody上 */
				Aris::DynKer::Marker *pU3j;/*!< \brief 指向坐标系U3j的指针，位于部件P3a上 */
				Aris::DynKer::Marker *pP1i;/*!< \brief 指向坐标系P1i的指针，位于部件Thigh上 */
				Aris::DynKer::Marker *pP1j;/*!< \brief 指向坐标系P1j的指针，位于部件P1a上 */
				Aris::DynKer::Marker *pP2i;/*!< \brief 指向坐标系P2i的指针，位于部件P2b上 */
				Aris::DynKer::Marker *pP2j;/*!< \brief 指向坐标系P2j的指针，位于部件P2a上 */
				Aris::DynKer::Marker *pP3i;/*!< \brief 指向坐标系P3i的指针，位于部件P3b上 */
				Aris::DynKer::Marker *pP3j;/*!< \brief 指向坐标系P3i的指针，位于部件P3a上 */
				Aris::DynKer::Marker *pS2i;/*!< \brief 指向坐标系S2i的指针，位于部件Thigh上 */
				Aris::DynKer::Marker *pS2j;/*!< \brief 指向坐标系S2j的指针，位于部件P2b上 */
				Aris::DynKer::Marker *pS3i;/*!< \brief 指向坐标系S3i的指针，位于部件Thigh上 */
				Aris::DynKer::Marker *pS3j;/*!< \brief 指向坐标系S3j的指针，位于部件P3b上 */
				Aris::DynKer::Marker *pSfi;/*!< \brief 指向坐标系Sfi的指针，位于部件Thigh上 */
				Aris::DynKer::Marker *pSfj;/*!< \brief 指向坐标系Sfj的指针，位于部件Ground上 */
			};
			Aris::DynKer::Marker *pMaks[19];
		};
		union
		{
			struct
			{
				Aris::DynKer::MotionBase *pM1;/*!< \brief 指向驱动M1的指针 */
				Aris::DynKer::MotionBase *pM2;/*!< \brief 指向驱动M2的指针 */
				Aris::DynKer::MotionBase *pM3;/*!< \brief 指向驱动M3的指针 */
			};

			Aris::DynKer::MotionBase *pMots[3];
		};
		union
		{
			struct
			{
				Aris::DynKer::SingleComponentForce *pF1;/*!< \brief 指向驱动M1的指针 */
				Aris::DynKer::SingleComponentForce *pF2;/*!< \brief 指向驱动M2的指针 */
				Aris::DynKer::SingleComponentForce *pF3;/*!< \brief 指向驱动M3的指针 */
			};

			Aris::DynKer::SingleComponentForce *pFces[3];
		};
		
	public:
		void GetdJacOverPee(double *dJi_x, double *dJi_y, double *dJi_z, const char *relativeCoordinate="G")const;

	private:
		LegI(const char *Name, RobotTypeI* pRobot);
		virtual ~LegI() = default;

		void FastDyn();
		
		void GetFin(double *fIn) const;
		void GetFinDyn(double *fIn) const;
		void GetFinFrc(double *fIn) const;

		virtual void calculate_from_pEE();
		virtual void calculate_from_pIn();
		virtual void calculate_from_vEE();
		virtual void calculate_from_vIn();
		virtual void calculate_from_aEE();
		virtual void calculate_from_aIn();

		virtual void calculate_jac();
		virtual void calculate_diff_jac();

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

	private:
		union
		{
			double fIn_dyn[3];
			struct
			{
				double f1_dyn;
				double f2_dyn;
				double f3_dyn;
			};
		};
		union
		{
			double fIn_frc[3];
			struct
			{
				double f1_frc;
				double f2_frc;
				double f3_frc;
			};
		};

	private:
		RobotTypeI *pRobot;

		const double U2x{ 0 }, U2y{ 0 }, U2z{ 0 }, U3x{ 0 }, U3y{ 0 }, U3z{ 0 };
		const double S2x{ 0 }, S2y{ 0 }, S2z{ 0 }, S3x{ 0 }, S3y{ 0 }, S3z{ 0 };
		const double Sfx{ 0 }, Sfy{ 0 }, Sfz{ 0 };

		const double D1{ 0 }, H1{ 0 }, D2{ 0 }, H2{ 0 };

		double a1, b1, va1, vb1, aa1, ab1;
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
		double inv_J1[3][3], inv_J2[3][3];

		double _C[36][36];
		double _c_M[36][4];

		friend class RobotTypeI;
	};
	class RobotTypeI :public Robots::RobotBase
	{
	public:
		RobotTypeI();
		~RobotTypeI() = default;
		
		virtual void LoadXml(const Aris::Core::XmlElement &xml_ele) override;
		using Model::LoadXml;

		void GetFin(double *Fin) const;
		void GetFinDyn(double *Fin) const;
		void GetFinFrc(double *Fin) const;

		void FastDyn();
		virtual void Dyn()override;

		void SetFixFeet(const char* fix_feet);
		const char* FixFeet() const;
		void SetActiveMotion(const char* active_motion);
		const char* ActiveMotion() const;

		
		
		struct SimResultNode
		{
			std::array<double, 6> Peb;
			std::array<double, 18> Pee;
			std::array<double, 18> Pin;
			std::array<double, 18> Fin;
		};
		void SimScriptClear();
		void SimScriptSetTopologyA();
		void SimScriptSetTopologyB();
		void SimScriptSimulate(std::uint32_t ms_dur, std::uint32_t ms_dt = 10);
		void SimPosCurve(const GaitFunc &fun, const GaitParamBase &param, std::list<SimResultNode> &result);
		void SimFceCurve(std::list<SimResultNode> &result, bool using_script = false);
		void SimMakeAkima(std::list<SimResultNode> &result, int ms_dt = 10);
		void SimByAdams(const std::string &adams_file, const GaitFunc &fun, const GaitParamBase &param, int ms_dt = 10, bool using_script = false);
		void SimByAdamsResultAt(int ms_time);
		void SimByMatlab(const std::string &folderName, const GaitFunc &fun, GaitParamBase &param, bool using_script = false);
	
	public:
		union
		{
			struct
			{
				LegI *const pLF;
				LegI *const pLM;
				LegI *const pLR;
				LegI *const pRF;
				LegI *const pRM;
				LegI *const pRR;
			};
			LegI *const pLegs[6];
		};

		Aris::DynKer::Marker* pBodyCenter;

	private:
		LegI LF_Leg{ "LF", this };
		LegI LM_Leg{ "LM", this };
		LegI LR_Leg{ "LR", this };
		LegI RF_Leg{ "RF", this };
		LegI RM_Leg{ "RM", this };
		LegI RR_Leg{ "RR", this };

		friend class LegI;
	};

	inline void Activate024(RobotTypeI *pRobot)
	{
		pRobot->pLF->pSf->Activate(false);
		pRobot->pLF->pM1->Activate(true);
		pRobot->pLF->pM2->Activate(true);
		pRobot->pLF->pM3->Activate(true);
		pRobot->pLF->pF1->Activate(false);
		pRobot->pLF->pF2->Activate(false);
		pRobot->pLF->pF3->Activate(false);

		pRobot->pLR->pSf->Activate(false);
		pRobot->pLR->pM1->Activate(true);
		pRobot->pLR->pM2->Activate(true);
		pRobot->pLR->pM3->Activate(true);
		pRobot->pLR->pF1->Activate(false);
		pRobot->pLR->pF2->Activate(false);
		pRobot->pLR->pF3->Activate(false);

		pRobot->pRM->pSf->Activate(false);
		pRobot->pRM->pM1->Activate(true);
		pRobot->pRM->pM2->Activate(true);
		pRobot->pRM->pM3->Activate(true);
		pRobot->pRM->pF1->Activate(false);
		pRobot->pRM->pF2->Activate(false);
		pRobot->pRM->pF3->Activate(false);

		pRobot->pLM->pSf->Activate(true);
		pRobot->pLM->pM1->Activate(false);
		pRobot->pLM->pM2->Activate(true);
		pRobot->pLM->pM3->Activate(true);
		pRobot->pLM->pF1->Activate(true);
		pRobot->pLM->pF2->Activate(false);
		pRobot->pLM->pF3->Activate(false);

		pRobot->pRF->pSf->Activate(true);
		pRobot->pRF->pM1->Activate(false);
		pRobot->pRF->pM2->Activate(true);
		pRobot->pRF->pM3->Activate(true);
		pRobot->pRF->pF1->Activate(true);
		pRobot->pRF->pF2->Activate(false);
		pRobot->pRF->pF3->Activate(false);

		pRobot->pRR->pSf->Activate(true);
		pRobot->pRR->pM1->Activate(false);
		pRobot->pRR->pM2->Activate(true);
		pRobot->pRR->pM3->Activate(true);
		pRobot->pRR->pF1->Activate(true);
		pRobot->pRR->pF2->Activate(false);
		pRobot->pRR->pF3->Activate(false);
	}
	inline void Activate135(RobotTypeI *pRobot)
	{
		pRobot->pLF->pSf->Activate(true);
		pRobot->pLF->pM1->Activate(false);
		pRobot->pLF->pM2->Activate(true);
		pRobot->pLF->pM3->Activate(true);
		pRobot->pLF->pF1->Activate(true);
		pRobot->pLF->pF2->Activate(false);
		pRobot->pLF->pF3->Activate(false);

		pRobot->pLR->pSf->Activate(true);
		pRobot->pLR->pM1->Activate(false);
		pRobot->pLR->pM2->Activate(true);
		pRobot->pLR->pM3->Activate(true);
		pRobot->pLR->pF1->Activate(true);
		pRobot->pLR->pF2->Activate(false);
		pRobot->pLR->pF3->Activate(false);

		pRobot->pRM->pSf->Activate(true);
		pRobot->pRM->pM1->Activate(false);
		pRobot->pRM->pM2->Activate(true);
		pRobot->pRM->pM3->Activate(true);
		pRobot->pRM->pF1->Activate(true);
		pRobot->pRM->pF2->Activate(false);
		pRobot->pRM->pF3->Activate(false);

		pRobot->pLM->pSf->Activate(false);
		pRobot->pLM->pM1->Activate(true);
		pRobot->pLM->pM2->Activate(true);
		pRobot->pLM->pM3->Activate(true);
		pRobot->pLM->pF1->Activate(false);
		pRobot->pLM->pF2->Activate(false);
		pRobot->pLM->pF3->Activate(false);

		pRobot->pRF->pSf->Activate(false);
		pRobot->pRF->pM1->Activate(true);
		pRobot->pRF->pM2->Activate(true);
		pRobot->pRF->pM3->Activate(true);
		pRobot->pRF->pF1->Activate(false);
		pRobot->pRF->pF2->Activate(false);
		pRobot->pRF->pF3->Activate(false);

		pRobot->pRR->pSf->Activate(false);
		pRobot->pRR->pM1->Activate(true);
		pRobot->pRR->pM2->Activate(true);
		pRobot->pRR->pM3->Activate(true);
		pRobot->pRR->pF1->Activate(false);
		pRobot->pRR->pF2->Activate(false);
		pRobot->pRR->pF3->Activate(false);
	}

}

#endif