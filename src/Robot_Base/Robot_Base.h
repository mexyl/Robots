#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <cstdint>
#include <vector>

#include "Aris_XML.h"
#include <Aris_DynModel.h>

namespace Robots
{
	class RobotBase;

	class LegBase:public Aris::DynKer::Object
	{
	public:
		using Marker = Aris::DynKer::Marker;
		using Part = Aris::DynKer::Part;

		const Part& Ground() const;
		Part& Ground();
		const Part& Body() const;
		Part& Body();
		const Marker& Base() const { return *pBase; };
		Marker& Base() { return *pBase; };

		void GetPee(double *pEE) const { GetPee(pEE, Ground()); };
		void GetPee(double *pEE, const Marker &Mak) const;
		void SetPee(const double *pEE) { SetPee(pEE, Ground()); };
		void SetPee(const double *pEE, const Marker &Mak);
		void GetVee(double *vEE) const { GetVee(vEE, Ground()); };
		void GetVee(double *vEE, const Marker &Mak) const;
		void SetVee(const double *vEE) { SetVee(vEE, Ground()); };
		void SetVee(const double *vEE, const Marker &Mak);
		void GetAee(double *aEE) const { GetAee(aEE, Ground()); };
		void GetAee(double *aEE, const Marker &Mak) const;
		void SetAee(const double *aEE) { SetAee(aEE, Ground()); };
		void SetAee(const double *aEE, const Marker &Mak);
		void GetFeeSta(double *fEE_sta) const { GetFeeSta(fEE_sta, Ground()); };
		void GetFeeSta(double *fEE_sta, const Marker &Mak) const;
		void SetFeeSta(const double *fEE_sta) { SetFeeSta(fEE_sta, Ground()); };
		void SetFeeSta(const double *fEE_sta, const Marker &Mak);
		/*!
		* \brief Get the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void GetPin(double *pIn) const;
		/*!
		* \brief Set the position of inputs
		* \param pIn position of inputs, double array with 3 elements
		*/
		void SetPin(const double *pIn);
		/*!
		* \brief Get the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void GetVin(double *vIn) const;
		/*!
		* \brief Set the velocity of inputs
		* \param vIn velocity of inputs, double array with 3 elements
		*/
		void SetVin(const double *vIn);
		/*!
		* \brief Get the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void GetAin(double *aIn) const;
		/*!
		* \brief Set the acceleration of inputs
		* \param aIn acceleration of inputs, double array with 3 elements
		*/
		void SetAin(const double *aIn);
		/*!
		* \brief Get the static actuation force of inputs, which is equal to Jfi * Fee_sta
		* \param fIn actuation force of inputs, double array with 3 elements
		*/
		void GetFinSta(double *fIn_sta) const;
		/*!
		* \brief Set the static actuation force of inputs, meanwhile the Fee_sta will be Jfd * Fin_sta
		* \param aIn actuation force of inputs, double array with 3 elements
		*/
		void SetFinSta(const double *fIn_sta);

		void GetJfd(double *jac) const { GetJfd(jac, Ground()); };
		void GetJfd(double *jac, const Marker &Mak) const;
		void GetJfi(double *jac) const { GetJfi(jac, Ground()); };
		void GetJfi(double *jac, const Marker &Mak) const;
		void GetJvd(double *jac) const { GetJvd(jac, Ground()); };
		void GetJvd(double *jac, const Marker &Mak) const;
		void GetJvi(double *jac) const { GetJvi(jac, Ground()); };
		void GetJvi(double *jac, const Marker &Mak) const;
		void GetDifJfd(double *dJac) const { GetDifJfd(dJac, Ground()); };
		void GetDifJfd(double *dJac, const Marker &Mak) const;
		void GetDifJfi(double *dJac) const { GetDifJfi(dJac, Ground()); };
		void GetDifJfi(double *dJac, const Marker &Mak) const;
		void GetDifJvd(double *dJac) const { GetDifJvd(dJac, Ground()); };
		void GetDifJvd(double *dJac, const Marker &Mak) const;
		void GetDifJvi(double *dJac) const { GetDifJvi(dJac, Ground()); };
		void GetDifJvi(double *dJac, const Marker &Mak) const;
		void GetCvd(double *c) const { GetCvd(c, Ground()); };
		void GetCvd(double *c, const Marker &Mak) const;
		void GetCvi(double *c) const { GetCvi(c, Ground()); };
		void GetCvi(double *c, const Marker &Mak) const;
		void GetCad(double *c) const { GetCad(c, Ground()); };
		/*!
		* \brief follow equation: Aee = Jvd * Ain + dJvd * Vin + Cad
		* \param c Cad
		*/
		void GetCad(double *c, const Marker &Mak) const;
		void GetCai(double *c) const { GetCai(c, Ground()); };
		/*!
		* \brief follow equation: Ain = Jvi * Aee + dJvi * Vee + Cai
		* \param c Cai
		*/
		void GetCai(double *c, const Marker &Mak) const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

	protected:
		LegBase(RobotBase* pRobot, const char *Name);
		virtual ~LegBase() = default;
		
		virtual void calculate_from_pEE(){};
		virtual void calculate_from_pIn(){};
		virtual void calculate_from_vEE(){};
		virtual void calculate_from_vIn(){};
		virtual void calculate_from_aEE(){};
		virtual void calculate_from_aIn(){};
		virtual void calculate_jac(){};
		virtual void calculate_diff_jac(){};

	protected:		
		RobotBase *pRobot;
		Aris::DynKer::Marker *pBase;

		union 
		{
			double pEE[3]{ 0 };
			struct
			{
				double x;
				double y;
				double z;
			};
		};
		union 
		{
			double vEE[3]{ 0 };
			struct
			{
				double vx;
				double vy;
				double vz;
			};
		};
		union 
		{
			double aEE[3]{ 0 };
			struct
			{
				double ax;
				double ay;
				double az;
			};
		};
		union 
		{
			double fEE_sta[3]{ 0 };
			struct
			{
				double fx_sta;
				double fy_sta;
				double fz_sta;
			};
		};
		union 
		{
			double pIn[3]{ 0 };
			struct
			{
				double l1;
				double l2;
				double l3;
			};
		};
		union 
		{
			double vIn[3]{ 0 };
			struct
			{
				double vl1;
				double vl2;
				double vl3;
			};
		};
		union 
		{
			double aIn[3]{ 0 };
			struct
			{
				double al1;
				double al2;
				double al3;
			};
		};
		union 
		{
			double fIn_sta[3]{ 0 };
			struct
			{
				double f1_sta;
				double f2_sta;
				double f3_sta;
			};
		};

		double _c_acc_dir[3]{ 0 };
		double _c_acc_inv[3]{ 0 };

		double Jvd[3][3]{ { 0 } };
		double Jvi[3][3]{ { 0 } };

		double vJvd[3][3]{ { 0 } };
		double vJvi[3][3]{ { 0 } };

		friend class RobotBase;
	};

	class RobotBase:public Aris::DynKer::Model
	{
	public:
		using Marker = Aris::DynKer::Marker;
		using Part = Aris::DynKer::Part;

		RobotBase();
		virtual ~RobotBase() = default;

		const Part& Ground() const { return *pGround; };
		Part& Ground() { return *pGround; };
		const Part& Body() const { return *pBody; };
		Part& Body() { return *pBody; };
		const Marker * const * const LegBases() const
		{ 
			static const Marker * const pBases[6]
			{
				pLegs[0]->pBase,
				pLegs[1]->pBase,
				pLegs[2]->pBase,
				pLegs[3]->pBase,
				pLegs[4]->pBase,
				pLegs[5]->pBase,
			};
			return pBases;
		};

		void GetPmb(double *pmb) const { GetPmb(pmb, Ground()); };
		void GetPmb(double *pmb, const Marker &mak) const;
		void SetPmb(const double *pmb) { SetPmb(pmb, Ground()); };
		void SetPmb(const double *pmb, const Marker &mak);
		void GetPeb(double *peb, const char *eurType = "313") const { GetPeb(peb, Ground(), eurType); };
		void GetPeb(double *peb, const Marker &mak, const char *eurType = "313") const;
		void SetPeb(const double *peb, const char *eurType = "313") { SetPeb(peb, Ground(), eurType); };
		void SetPeb(const double *peb, const Marker &mak, const char *eurType = "313");
		void GetPqb(double *pqb) const { GetPqb(pqb, Ground()); };
		void GetPqb(double *pqb, const Marker &mak) const;
		void SetPqb(const double *pqb) { SetPqb(pqb, Ground()); };
		void SetPqb(const double *pqb, const Marker &mak);
		void GetVb(double *vb) const { GetVb(vb, Ground()); };
		void GetVb(double *vb, const Marker &mak) const;
		void SetVb(const double *vb) { SetVb(vb, Ground()); };
		void SetVb(const double *vb, const Marker &mak);
		void GetAb(double *ab) const { GetAb(ab, Ground()); };
		void GetAb(double *ab, const Marker &mak) const;
		void SetAb(const double *ab) { SetAb(ab, Ground()); };
		void SetAb(const double *ab, const Marker &mak);

		void GetPee(double *pEE) const { GetPee(pEE, Ground()); };
		void GetPee(double *pEE, const Marker &Mak) const;
		void SetPee(const double *pEE) { SetPee(pEE, Ground()); };
		void SetPee(const double *pEE, const Marker &Mak);
		void GetVee(double *vEE) const { GetVee(vEE, Ground()); };
		void GetVee(double *vEE, const Marker &Mak) const;
		void SetVee(const double *vEE) { SetVee(vEE, Ground()); };
		void SetVee(const double *vEE, const Marker &Mak);
		void GetAee(double *aEE) const { GetAee(aEE, Ground()); };
		void GetAee(double *aEE, const Marker &Mak) const;
		void SetAee(const double *aEE) { SetAee(aEE, Ground()); };
		void SetAee(const double *aEE, const Marker &Mak);
		void GetFeeSta(double *fEE_sta) const { GetFeeSta(fEE_sta, Ground()); };
		void GetFeeSta(double *fEE_sta, const Marker &Mak) const;
		void SetFeeSta(const double *fEE_sta) { SetFeeSta(fEE_sta, Ground()); };
		void SetFeeSta(const double *fEE_sta, const Marker &Mak);
		
		void GetPee(double *pEE, const Marker* const pMaks[]) const;
		void SetPee(const double *pEE, const Marker* const pMaks[]);
		void GetVee(double *vEE, const Marker* const pMaks[]) const;
		void SetVee(const double *vEE, const Marker* const pMaks[]);
		void GetAee(double *aEE, const Marker* const pMaks[]) const;
		void SetAee(const double *aEE, const Marker* const pMaks[]);
		void GetFeeSta(double *fee_sta, const Marker* const pMaks[]) const;
		void SetFeeSta(const double *fee_sta, const Marker* const pMaks[]);
		
		void GetPin(double *pIn) const;
		void SetPin(const double *pIn);
		void GetVin(double *vIn) const;
		void SetVin(const double *pIn);
		void GetAin(double *aIn) const;
		void SetAin(const double *pIn);
		void GetFinSta(double *fIn_sta) const;
		void SetFinSta(const double *fIn_sta);

		/*!
		* \brief 根据雅可比矩阵和初值迭代求解，主要用来在已知足端位置和输入位置时，机器人身体的位置。
		* 对于固定于地面的腿，对其SetPee，然后迭代求解身体位姿，之后对不在地面上的腿SetPin
		*/
		void SetPinFixFeet(const double *pIn, const char *fixFeet, const char *activeMotor, const double *initBodyPE);
		void SetVinFixFeet(const double *vIn, const char *fixFeet, const char *activeMotor);
		void SetAinFixFeet(const double *aIn, const char *fixFeet, const char *activeMotor);

		void GetJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJfd(double *jac_out, const char *activeMotion = "111111111111111111") const;
		void GetDifJvi(double *jac_out, const char *activeMotion = "111111111111111111") const;

		void TransformCoordinatePee(const double *bodyPe, const char *fromMak, const double *fromPee
			, const char *toMak, double *toPee) const;

		virtual void LoadXml(const char *file) { this->Model::LoadXml(file); };
		virtual void LoadXml(const Aris::Core::XmlDocument &doc) { this->Model::LoadXml(doc); };

		LegBase *pLegs[6];

	protected:
		Aris::DynKer::Part *pBody;

	private:
		double _BodyPm[4][4]{ {0} }, _BodyVel[6]{ 0 }, _BodyAcc[6]{ 0 };
		double Jvi[18][6]{ {0} };
		double vJvi[18][6]{ {0} };
		void calculate_jac();
		void calculate_jac_c();

		friend class LegBase;
	};
}

#endif
