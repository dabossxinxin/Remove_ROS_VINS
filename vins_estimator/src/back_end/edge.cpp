#include <iostream>
#include "edge.h"
#include "vertex.h"

namespace backend {
	unsigned long global_edge_id = 0;

	Edge::Edge(int residual_dimension, int verticies_num,
		const std::vector<std::string>& types) 
	{
		residual.resize(residual_dimension, 1);
		if (!verticies_types.empty()) {
			verticies_types = types;
		}

		edge_id = global_edge_id++;
		MatXXd info(residual_dimension, residual_dimension);
		info.setIdentity();
		information = info;
		sqrt_information = info;
		loss_function = nullptr;
	}

	Edge::~Edge()
	{
	}

	double Edge::Chi2() const
	{
		return residual.transpose()*information*residual;
	}

	double Edge::RobustChi2() const 
	{
		double ee = this->Chi2();
		if (loss_function) {
			Eigen::Vector3d rho;
			loss_function->Compute(ee, rho);
			ee = rho[0];
		}
		return ee;
	}

	void Edge::RobustInfo(double& drho, MatXXd& info) const
	{
		if (loss_function) {
			double ee = this->Chi2();
			Eigen::Vector3d rho;
			loss_function->Compute(ee, rho);
			VecXd weight_err = sqrt_information * residual;
			
			MatXXd robust_info(information.rows(), information.cols());
			robust_info.setIdentity();
			robust_info *= rho[1];
			
			if (rho[1] + 2 * rho[2] * ee > 0) {
				robust_info += 2 * rho[2] * weight_err*weight_err.transpose();
			}

			info = robust_info * information;
			drho = rho[1];
		}
		else {
			drho = 1.0;
			info = information;
		}
	}

	bool Edge::CheckValid()
	{
		if (!verticies_types.empty()) {
			for (int it = 0; it < static_cast<int>(verticies.size()); ++it) {
				if (verticies_types[it] != verticies[it]->TypeInfo()) {
					std::cout << "Vertex types does not match, should be "
						<< verticies_types[it] << ", but set to "
						<< verticies[it]->TypeInfo() << std::endl;
				}
			}
		}

		if (information.rows() != information.cols()) {
			std::cout << "Edge information matrix dimension error" << std::endl;
		}

		if (residual.rows() != information.rows()) {
			std::cout << "Resduals or information dimension error in Edge" << std::endl;
		}

		if (residual.rows() != observation.rows()) {
			std::cout << "Residuals or observation dimansion error in Edge" << std::endl;
		}

		for (int it = 0; it < static_cast<int>(jacobians.size()); +it) {
			if (jacobians[it].rows() != residual.rows()) {
				std::cout << "Jacobians or residual dimansion error in Edge" << std::endl;
			}

			if (jacobians[it].cols() != verticies[it]->LocalDimension()) {
				std::cout << "Jacobians or residual dimansion error in Edge" << std::endl;
			}
		}
	}
}