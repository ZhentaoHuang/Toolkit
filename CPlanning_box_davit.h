#ifndef _TEST_H_
#define _TEST_H_


#include <vector>
#include <time.h>
#include <stack>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <math.h>
#include <set>
#include<errno.h>

using namespace std;



namespace GAPacking{



const int LEFT = 1;
const int RIGHT = 2;
const int DOWN = 3;
const int TOP = 4;

const int LEFT_DOWN = 5;
const int LEFT_TOP = 6;
const int RIGHT_DOWN = 7;
const int RIGHT_TOP = 8;

const int Min_Box_Size = 22; //最小箱子最小边长
const int Max_Allowed_Gap_Size = 5;  //可接受的不可用缝隙最大边长
const int available_gaps_num_limit = 20; //每次判断放置位置时最多比较的箱子数量
const int Cave_Depth = 15;

const float stability_threshold_area = 0.4; //箱子在支撑面积上的容许程度
const float stability_threshold_center = 0.6;//箱子在中心偏移上的容许程度
const int allow_z_err = 2; //箱子判断支撑底面时的高度容差
const int z_allowed_over_pallet = 5; //允许箱子超过龙车的高度


// 用于遗传算法的权重参数
const float weight_height = 0.94 * 10;
const float weight_range = 0.74;
const float weight_support = 0.16;
const float weight_area_supported_area = 0.98;
const float weight_area_supported_center = 0.16;
const float weight_area_supported_volume = 0.16;
const float weight_area_contacted = 0.74;
const float weight_area_created = 0.94;
const float weight_area_corner = 0.94;

//高度图网格单元,height表示该格高度
struct grid
{
	int Height;
 	int Grid_x, Grid_y;
};

//箱子信息,packst表示此箱子是否被放入托盘,dim1, dim2, dim3表示箱子尺寸,cox, coy, coz,表示箱子放置位置,packx, packy, packz表示箱子放置姿态（即是否旋转）
struct boxinfo
{
	int packst;
	int dim1, dim2, dim3, cox, coy, coz, packx, packy, packz;
	long int vol;
};

//记录空隙信息的基础单位，分别记录了空隙的左、右、上、下以及处于的Z值，包括空隙的面积
struct gap_range
{
	int left;
	int right;
	int down;
	int top;
	int z_dim;
	int area;
};
//记录选择的空隙的高级信息，z表示空隙高度，index表示该空隙在z高度下所有空隙中的编号，orientation表示箱子是否旋转，position表示箱子紧贴空隙的哪个角放置
struct gap_info
{
	int z;
	int index;
	int box_orientation;
    int box_position = LEFT_DOWN;
};



class CPlanning_Box
{
    public:
    CPlanning_Box(int px, int py, int pz);//初始化
    int Reset();//清空，初始化所有参数
    
    bool PlacementPlanning(boxinfo &box);//将给定箱子放入当前托盘，输出能否放置

    int UpdateState(boxinfo &box); //可以放置箱子的话，放置箱子并更新高度图
    int UpdateStateByVision(vector <vector<int> > &Ranges);
    int SavePlacement(boxinfo &box);
    int SaveState();

    private:
    int ClearPallet(grid **Height_Map,int px, int py, int pz,bool have_wall);//清空托盘
    
    int FindGapsOnZ(int z); //寻找在高度z上的所有空隙，将所有空隙放入 gaps-set【z】
    int FindRectangleArea(vector<vector<int>> available_matirx,int z); //寻找高度z空间内所有有支撑的正方形空隙
    
    
    vector<gap_info> FindGapsAvailable(boxinfo box); //在gaps-set中由低到高寻找前x个能够放入箱子的空隙（包括旋转箱子的）
    gap_info FindBestGap(boxinfo box, vector<gap_info> gap_solutions); //计算给定空隙集中所有空隙的分数，选取最高分空隙输出
    float EvaluateGap(boxinfo box,gap_info &gap_solution);  //在给定单个空隙中寻找最好的放置位置（角落），并给出评分
    
    float FindBestPosition(boxinfo box, gap_range gap,gap_info &gap_solution);//计算给定空隙所有放置位置（角落）评分，选择最好的输出
    float EvaluateAreaSupported(boxinfo box,gap_range gap); //计算给定角落支撑评分
    float EvaluateAreaContacted(boxinfo box,gap_range gap);//计算给定角落接触评分
    float EvaluateAreaCreated(boxinfo box,gap_range gap); //计算给定角落贡献评分
    float EvaluateAreaCorner(boxinfo box,gap_range gap);
    float EvaluateHeight(boxinfo box, gap_range gap); //计算单个空隙高度评分
    float EvaluateRange(boxinfo box, gap_range gap); //计算单个空隙范围评分
    float EvaluateSupport(boxinfo box, gap_range gap); //计算单个空隙支撑评分
    int FindIslandArea(int x,int y);//寻找01矩阵中包含点xy的最大面积
    bool CaveDetection(int box_x,int box_y, gap_range gap, int cave_depth);

    int CalculBoxCoordinate(boxinfo &box,gap_info best_gap); //计算箱子放置具体坐标
    float CalculOccupacyRatio(boxinfo &box); //计算容积率
    
    int UpdateHeightMap(boxinfo &box); //更新高度图
    int UpdateGaps(int highest_height_to_update, int lowest_height_to_update); //更新可用空隙信息
    

    public:
    int pallet_x,pallet_y,pallet_z; //托盘大小
    int box_count; //箱子计数
    int pallet_volume; //托盘容积
    int packed_volume;  //放置箱子总容积
    float volume_ratio; //容积率
    bool pallet_is_full; //托盘是否已满
    int find_low_gap = 60; //设置寻找最低层的范围
    bool have_wall = true; //是否有墙壁（是龙车）

    vector<boxinfo> BoxType;  //箱子信息
    grid ** Height_Map;  //高度图矩阵
    vector<vector<gap_range>> gaps_set; //空隙记录二维矩阵，第一位z代表托盘高度，第二位index代表z高度下的所有空隙
    set<int> gaps_height; //当前托盘可用来放置箱子的所有高度，堆结构，可自动排序
    vector<int> temp_gaps_height; //用于缓存高度信息的临时向量
    vector<boxinfo> packed_boxes; //用于存储所有已经放置箱子信息的箱子序列向量
    vector<vector<int>> island_matrix;  //用于寻找 max island的临时矩阵

 



     

};



};




#endif