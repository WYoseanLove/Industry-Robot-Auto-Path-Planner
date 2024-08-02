#pragma once
#include <cmath>
#include <iostream>
#include <random>
#include <limits>
#include <ctime>


#define lib_export
#ifdef lib_export
#define cs_lib_api extern "C" __declspec(dllexport)
#else
#define cs_lib_api __declspec(dllimport)
#endif

#ifdef DLLRANDOMCPP_EXPORTS
#define DLLRANDOMCPP_API __declspec(dllexport)
#else
#define DLLRANDOMCPP_API __declspec(dllimport)
#endif



std::random_device goal_rd_; //goal_rd_() 随机生成一个数字；这里random_device类没有设置随机生成数字的范围；

std::mt19937 goal_gen_; //生成随机数
std::uniform_int_distribution<int> goal_dis_;

std::random_device area_rd_;//area_rd_() 随机生成一个数字；这里random_device类没有设置随机生成数字的范围；
std::mt19937 area_gen_;//生成随机数；
std::uniform_real_distribution<double> area_dis_;


bool random_inital();
int generate_goal_dis();
double generate_area_dis();
double numberLimitMax_double();
int generate_rand_Int();

extern "C" DLLRANDOMCPP_API bool RANDOM_Init() { return random_inital(); }

extern "C" DLLRANDOMCPP_API int GenerateGoal_dis() { return generate_goal_dis(); }

extern "C" DLLRANDOMCPP_API double GenerateArea_dis() { return generate_area_dis(); }

extern "C" DLLRANDOMCPP_API double Number_Limit_MAX() { return numberLimitMax_double(); }

extern "C" DLLRANDOMCPP_API int Generate_rand_Int() { return generate_rand_Int(); }

