#include "Force.h"

#include <iostream>

void ConstantForce::addAcceleration(const VectorXd &x, const VectorXd &v, VectorXd &a) {
	int nodeSize = a.size()/numNodes;
	for (int i = 0; i < numNodes; i++) {
		a[i*nodeSize + positionOffset + 0] += vec[0];
		a[i*nodeSize + positionOffset + 1] += vec[1];
		a[i*nodeSize + positionOffset + 2] += vec[2];
	}
}

void ConstantForce::addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv) {
	/*int nodeSize = x.size()/numNodes;
	for (int i = 0; i < numNodes; i++) {
		for (int j = 0; j < 3; j++) {
			Jv(nodeSize*i + positionOffset + j, nodeSize*i + positionOffset + j) += vec[j];
		}
	}*/
}

void AnchorForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
	Vector3d dx = x.segment(nodeSize*node + positionOffset, 3) - Vector3d(anchorPoint[0], anchorPoint[1], anchorPoint[2]);
	f.segment(nodeSize*node + positionOffset, 3) += -ks*dx - kd*v.segment(nodeSize*node + positionOffset, 3);
}

void AnchorForce::addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv) {
	int nodeSize = x.size()/numNodes;
	Jx.block(nodeSize*node + positionOffset,nodeSize*node + positionOffset, 3,3) += -ks*Matrix2d::Identity();
    Jv.block(nodeSize*node + positionOffset,nodeSize*node + positionOffset, 3,3) += -kd*Matrix2d::Identity();
}

void SpringForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
  	Vector3d l = x.segment(nodeSize*node1 + positionOffset, 3) - x.segment(nodeSize*node2 + positionOffset, 3);
	Vector3d dl = v.segment(nodeSize*node1 + positionOffset, 3) - v.segment(nodeSize*node2 + positionOffset, 3);
	f.segment(nodeSize*node1 + positionOffset, 3) += -(ks*(l.norm() - l0) + kd*dl.dot(l/l.norm()))*l/l.norm();
	f.segment(nodeSize*node2 + positionOffset, 3) += (ks*(l.norm() - l0) + kd*dl.dot(l/l.norm()))*l/l.norm();
}

void SpringForce::addJacobians(const VectorXd &x, const VectorXd &v, MatrixXd &Jx, MatrixXd &Jv) {
	int nodeSize = x.size()/numNodes;
  	Vector3d l = x.segment(nodeSize*node2 + positionOffset, 3) - x.segment(nodeSize*node1 + positionOffset, 3);

  	Vector3d ln = l/l.norm();
  	Matrix3d fx = -ks*(glm::max((1.0-l0/l.norm()),0.0)*(Matrix3d::Identity()-ln*ln.transpose()) + ln*ln.transpose());
  	Matrix3d vx = -kd*ln*ln.transpose();

  	if (record) {
  		//std::cout << node1 << std::endl;
  		//std::cout << ln[0] << " " << ln[1] << " " << ln[2] << std::endl;
  		/*for (int i = 0; i < 3; i++) {
  			for (int j = 0; j < 3; j++) {
  				std::cout << fx(i,j) << " ";
  			}
  			std::cout  << std::endl;
  		}
  		std::cout << std::endl;*/
  	}

  	Jx.block(nodeSize*node1 + positionOffset,nodeSize*node1 + positionOffset, 3,3) += fx;
	Jx.block(nodeSize*node1 + positionOffset,nodeSize*node2 + positionOffset, 3,3) -= fx;
	Jx.block(nodeSize*node2 + positionOffset,nodeSize*node1 + positionOffset, 3,3) -= fx;
	Jx.block(nodeSize*node2 + positionOffset,nodeSize*node2 + positionOffset, 3,3) += fx;

  	Jv.block(nodeSize*node1 + positionOffset,nodeSize*node1 + positionOffset, 3,3) += vx;
  	Jv.block(nodeSize*node1 + positionOffset,nodeSize*node2 + positionOffset, 3,3) -= vx;
  	Jv.block(nodeSize*node2 + positionOffset,nodeSize*node1 + positionOffset, 3,3) -= vx;
  	Jv.block(nodeSize*node2 + positionOffset,nodeSize*node2 + positionOffset, 3,3) += vx;


  	/*l = x.segment(nodeSize*node2 + positionOffset, 3) - x.segment(nodeSize*node1 + positionOffset, 3);
  	ln = l/l.norm();
  	fx = -ks*(glm::max((1.0-l0/l.norm()),0.0)*(Matrix3d::Identity()-ln*ln.transpose()) + ln*ln.transpose());
  	vx = -kd*ln*ln.transpose();

  	Jx.block(nodeSize*node2 + positionOffset,nodeSize*node2 + positionOffset, 3,3) += fx;
  	Jv.block(nodeSize*node2 + positionOffset,nodeSize*node2 + positionOffset, 3,3) += vx;
  	
  	
 /* Vector2d l = p0->x - p1->x;
  Vector2d ln = l/l.norm();
  Matrix2d fx = -ks*(max((1.0-l0/l.norm()),0.0)*(Matrix2d::Identity()-ln*ln.transpose()) + ln*ln.transpose());
  Matrix2d vx = -kd*ln*ln.transpose();

  Jx.block(2*p0->i,2*p0->i, 2,2) += fx;
  Jv.block(2*p0->i,2*p0->i, 2,2) += vx;

  l = p1->x - p0->x;
  ln = l/l.norm();
  fx = -ks*(max((1.0-l0/l.norm()),0.0)*(Matrix2d::Identity()-ln*ln.transpose()) + ln*ln.transpose());
  vx = -kd*ln*ln.transpose();

  Jx.block(2*p1->i,2*p1->i, 2,2) += fx;
  Jv.block(2*p1->i,2*p1->i, 2,2) += vx;*/
}

void AreoForce::addForces(const VectorXd &x, const VectorXd &v, VectorXd &f) {
	int nodeSize = f.size()/numNodes;
	Vector3d r1 = x.segment(nodeSize*nodes[0] + positionOffset, 3);
	Vector3d r2 = x.segment(nodeSize*nodes[1] + positionOffset, 3);
	Vector3d r3 = x.segment(nodeSize*nodes[2] + positionOffset, 3);
	Vector3d v1 = v.segment(nodeSize*nodes[0] + positionOffset, 3);
	Vector3d v2 = v.segment(nodeSize*nodes[1] + positionOffset, 3);
	Vector3d v3 = v.segment(nodeSize*nodes[2] + positionOffset, 3);

	//airV += glm::vec3(0.001,0.0,0.0);

	Vector3d vNew = (v1 + v2 + v3)/3.0 - Vector3d(airV[0], airV[1], airV[2]);

	Vector3d n = (r2-r1).cross(r3-r1);
	Vector3d v2an = (vNew.norm()*(vNew.dot(n))/(2*n.norm()))*n;
	Vector3d fAreo = (-0.5)*density*cd*v2an;
	f.segment(nodeSize*nodes[0] + positionOffset, 3) += (1.0/3.0)*fAreo;
	f.segment(nodeSize*nodes[1] + positionOffset, 3) += (1.0/3.0)*fAreo;
	f.segment(nodeSize*nodes[2] + positionOffset, 3) += (1.0/3.0)*fAreo;
}