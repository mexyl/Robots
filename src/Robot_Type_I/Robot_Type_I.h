#ifndef ROBOT_III_H
#define ROBOT_III_H

#include <aris.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>

namespace Robots
{
	class RobotTypeI;
	class LegI :public Robots::LegBase
	{
	public:
		auto partAt(std::size_t id)->Aris::Dynamic::Part& { return *prt_array_[id]; };
		auto markerAt(std::size_t id)->Aris::Dynamic::Marker& { return *mak_array_[id]; };
		auto jointAt(std::size_t id)->Aris::Dynamic::Joint& { return *jnt_array_[id]; };
		auto motionAt(std::size_t id)->Aris::Dynamic::Motion& { return *mot_array_[id];};
		auto forceAt(std::size_t id)->Aris::Dynamic::SingleComponentForce& { return static_cast<Aris::Dynamic::SingleComponentForce&>(*fce_array_[id]); };

		auto p1a()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*p1a_); };
		auto p2a()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*p2a_); };
		auto p3a()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*p3a_); };
		auto thigh()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*thigh_); };
		auto p2b()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*p2b_); };
		auto p3b()->Aris::Dynamic::Part& { return static_cast<Aris::Dynamic::Part&>(*p3b_); };

		auto u1i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u1i_); };
		auto u1j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u1j_); };
		auto u2i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u2i_); };
		auto u2j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u2j_); };
		auto u3i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u3i_); };
		auto u3j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*u3j_); };
		auto p1i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p1i_); };
		auto p1j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p1j_); };
		auto p2i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p2i_); };
		auto p2j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p2j_); };
		auto p3i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p3i_); };
		auto p3j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*p3j_); };
		auto sfi()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*sfi_); };
		auto sfj()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*sfj_); };
		auto s2i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*s2i_); };
		auto s2j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*s2j_); };
		auto s3i()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*s3i_); };
		auto s3j()->Aris::Dynamic::Marker& { return static_cast<Aris::Dynamic::Marker&>(*s3j_); };
		
		auto u1()->Aris::Dynamic::UniversalJoint& { return static_cast<Aris::Dynamic::UniversalJoint&>(*u1_); };
		auto u2()->Aris::Dynamic::UniversalJoint& { return static_cast<Aris::Dynamic::UniversalJoint&>(*u2_); };
		auto u3()->Aris::Dynamic::UniversalJoint& { return static_cast<Aris::Dynamic::UniversalJoint&>(*u3_); };
		auto p1()->Aris::Dynamic::TranslationalJoint& { return static_cast<Aris::Dynamic::TranslationalJoint&>(*p1_); };
		auto p2()->Aris::Dynamic::TranslationalJoint& { return static_cast<Aris::Dynamic::TranslationalJoint&>(*p2_); };
		auto p3()->Aris::Dynamic::TranslationalJoint& { return static_cast<Aris::Dynamic::TranslationalJoint&>(*p3_); };
		auto sf()->Aris::Dynamic::SphericalJoint& { return static_cast<Aris::Dynamic::SphericalJoint&>(*sf_); };
		auto s2()->Aris::Dynamic::SphericalJoint& { return static_cast<Aris::Dynamic::SphericalJoint&>(*s2_); };
		auto s3()->Aris::Dynamic::SphericalJoint& { return static_cast<Aris::Dynamic::SphericalJoint&>(*s3_); };
		auto m1()->Aris::Dynamic::SingleComponentMotion& { return static_cast<Aris::Dynamic::SingleComponentMotion&>(*m1_); };
		auto m2()->Aris::Dynamic::SingleComponentMotion& { return static_cast<Aris::Dynamic::SingleComponentMotion&>(*m2_); };
		auto m3()->Aris::Dynamic::SingleComponentMotion& { return static_cast<Aris::Dynamic::SingleComponentMotion&>(*m3_); };
		auto f1()->Aris::Dynamic::SingleComponentForce& { return static_cast<Aris::Dynamic::SingleComponentForce&>(*f1_); };
		auto f2()->Aris::Dynamic::SingleComponentForce& { return static_cast<Aris::Dynamic::SingleComponentForce&>(*f2_); };
		auto f3()->Aris::Dynamic::SingleComponentForce& { return static_cast<Aris::Dynamic::SingleComponentForce&>(*f3_); };
		
		
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
		typedef Aris::Dynamic::Part* PartPtr;
		union
		{
			PartPtr prt_array_[6];
			struct
			{
				PartPtr p1a_, p2a_, p3a_, thigh_, p2b_, p3b_;
			};
		};
		typedef Aris::Dynamic::Marker* MarkerPtr;
		union
		{
			MarkerPtr mak_array_[18];
			struct
			{
				MarkerPtr u1i_, u1j_, u2i_, u2j_, u3i_, u3j_;
				MarkerPtr p1i_, p1j_, p2i_, p2j_, p3i_, p3j_;
				MarkerPtr sfi_, sfj_, s2i_, s2j_, s3i_, s3j_;
			};
		};

		typedef Aris::Dynamic::Joint* JointPtr;
		union
		{
			JointPtr jnt_array_[9];
			struct
			{
				JointPtr u1_, u2_, u3_, p1_, p2_, p3_, sf_, s2_, s3_;
			};
		};
		typedef Aris::Dynamic::Motion* MotionPtr;
		union
		{
			MotionPtr mot_array_[3];
			struct
			{
				MotionPtr m1_, m2_, m3_;
			};
		};
		typedef Aris::Dynamic::Force* ForcePtr;
		union
		{
			ForcePtr fce_array_[3];
			struct
			{
				ForcePtr f1_, f2_, f3_;
			};
		};
		
		
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
		
		virtual auto loadXml(const Aris::Core::XmlElement &xml_ele)->void override;
		virtual auto saveXml(Aris::Core::XmlElement &xml_ele)const->void override;
		using Model::loadXml;
		using Model::saveXml;

		void GetFin(double *Fin) const;
		void GetFinDyn(double *Fin) const;
		void GetFinFrc(double *Fin) const;

		void FastDyn();
		virtual void dyn()override;

		void SetFixFeet(const char* fix_feet);
		const char* FixFeet() const;
		void SetActiveMotion(const char* active_motion);
		const char* ActiveMotion() const;

		virtual void kinFromPin()override
		{
			double Pin[18];
			for (int i = 0; i < 18; ++i)
			{
				Pin[i] = motionPool().at(i).motPos();
			}

			double pe[6];
			this->GetPeb(pe);
			SetPinFixFeet(Pin, FixFeet(), ActiveMotion(), pe);
		};
		virtual void kinFromVin()override
		{
			double Vin[18];
			for (int i = 0; i < 18; ++i)
			{
				Vin[i] = motionPool().at(i).motVel();
			}
			SetVinFixFeet(Vin, FixFeet(), ActiveMotion());
		};
		auto simToAdams(const std::string &adams_file, const Aris::Dynamic::PlanFunc &fun, const Aris::Dynamic::PlanParamBase &param, int ms_dt)->Aris::Dynamic::SimResult;
	
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

	private:
		LegI LF_Leg{ "LF", this };
		LegI LM_Leg{ "LM", this };
		LegI LR_Leg{ "LR", this };
		LegI RF_Leg{ "RF", this };
		LegI RM_Leg{ "RM", this };
		LegI RR_Leg{ "RR", this };

		friend class LegI;
	};


}

#endif