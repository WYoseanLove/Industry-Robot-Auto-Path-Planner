#include "pch.h"
#include "randomcpp.h"
double numberLimitMax_double()
{
	return std::numeric_limits<double>::max();
	
}


bool random_inital()
{
	goal_gen_ = std::mt19937(goal_rd_());
	goal_dis_ = std::uniform_int_distribution<int>(0, 100); //生成0-100之间均匀分布的随机数,数据类型为int；
	area_gen_ = std::mt19937(area_rd_());
	area_dis_ = std::uniform_real_distribution<double>(0, 15);
	srand(time(NULL));
	return true;
}

int generate_goal_dis()
{
	int x = goal_dis_(goal_gen_);
	return x;
}

double generate_area_dis()
{
	double y = area_dis_(goal_gen_);
	return y;
}

int generate_rand_Int()
{
	return rand();


}

