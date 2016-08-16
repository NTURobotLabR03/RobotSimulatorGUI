/*
//類別名稱：無
//作者：Liu Yi-Ren
//日期：2016/08/15
//目的：Eigen Library的所有include檔
//      以及輔助functino
//使用函式庫：Eigen
*/
#pragma once
#include <Eigen\Dense>
#include <Eigen\StdVector>
#include <unsupported\Eigen\Splines>

//將a向量轉到b向量所需的rotation matrix
void a2bRotation(const Eigen::Vector3d& a, const Eigen::Vector3d&b, Eigen::Matrix3d& R);
//把kaxis轉Th表示試轉回Rotation matrix
void kaxisTh2Rot(const Eigen::Vector3d& o, Eigen::Matrix3d& R);
//給與一個法向量n 會回傳兩個切向量 存入t裡面
void findPlaneVectors(Eigen::Vector3d& n, std::vector < Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& t);