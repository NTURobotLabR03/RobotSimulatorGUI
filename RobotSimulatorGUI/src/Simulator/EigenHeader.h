/*
//���O�W�١G�L
//�@�̡GLiu Yi-Ren
//����G2016/08/15
//�ت��GEigen Library���Ҧ�include��
//      �H�λ��Ufunctino
//�ϥΨ禡�w�GEigen
*/
#pragma once
#include <Eigen\Dense>
#include <Eigen\StdVector>
#include <unsupported\Eigen\Splines>

//�Na�V�q���b�V�q�һݪ�rotation matrix
void a2bRotation(const Eigen::Vector3d& a, const Eigen::Vector3d&b, Eigen::Matrix3d& R);
//��kaxis��Th��ܸ���^Rotation matrix
void kaxisTh2Rot(const Eigen::Vector3d& o, Eigen::Matrix3d& R);
//���P�@�Ӫk�V�qn �|�^�Ǩ�Ӥ��V�q �s�Jt�̭�
void findPlaneVectors(Eigen::Vector3d& n, std::vector < Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& t);