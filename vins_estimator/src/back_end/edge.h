#pragma once

#include "eigen_types.h"
#include "loss_function.h"

namespace backend {

	class Vertex;

	class Edge {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit Edge(int residual_dimension, int verticies_num,
			const std::vector<std::string>& types = std::vector<std::string>());

		virtual ~Edge();

		unsigned long Id() const 
		{
			return edge_id;
		}

		bool AddVertex(std::shared_ptr<Vertex>& vertex) {
			verticies.emplace_back(vertex);
			return true;
		}

		bool SetVertex(const std::vector<std::shared_ptr<Vertex>>& vertex) 
		{
			verticies = vertex;
			return true;
		}

		std::shared_ptr<Vertex> GetVertex(int id) 
		{
			return verticies[id];
		}

		std::vector<std::shared_ptr<Vertex>> Verticies() const 
		{
			return verticies;
		}

		size_t VerticesNum() const 
		{
			return verticies.size();
		}

		virtual std::string TypeInfo() const = 0;
		
		virtual void ComputeResidual() = 0;

		virtual void ComputeJacobians() = 0;

		double Chi2() const;

		double RobustChi2() const;

		VecXd Residual() const
		{
			return residual;
		}

		VecMatXXd Jacobians() const 
		{
			return jacobians;
		}

		void SetInformatio(const MatXXd& info)
		{
			information = info;
			sqrt_information = Eigen::LLT<MatXXd>(information).matrixL().transpose();
		}

		MatXXd Information() const 
		{
			return information;
		}

		MatXXd SqrtInformation() const
		{
			return sqrt_information;
		}

		void SetLossFunction(LossFunction* loss) 
		{
			loss_function = loss;
		}

		LossFunction* GetLossFunction() 
		{
			return loss_function;
		}

		void RobustInfo(double& drho, MatXXd& info) const;

		void SetObservation(const VecXd& obs)
		{
			observation = obs;
		}

		VecXd Observation() const
		{
			return observation;
		}

		bool CheckValid();

		unsigned long OrderedId() const
		{
			return edge_ordered_id;
		}

		void SetOrderedId(unsigned long id)
		{
			edge_ordered_id = id;
		}

	protected:
		unsigned long	edge_id;
		unsigned long	edge_ordered_id;
		LossFunction	*loss_function;

		std::vector<std::string>				verticies_types;
		std::vector<std::shared_ptr<Vertex>>	verticies;
		
		VecXd			residual;	// ²Ð²î=Ô¤²â-¹Û²â
		VecXd			observation;
		MatXXd			information;
		MatXXd			sqrt_information;
		VecMatXXd		jacobians;
	};
}