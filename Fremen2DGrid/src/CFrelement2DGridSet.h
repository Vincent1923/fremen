#ifndef CFRELEMENT2DGRIDSET_H
#define CFRELEMENT2DGRIDSET_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "CFrelement2DGrid.h"
#include <nav_msgs/OccupancyGrid.h>

#define MAX_LENGTH 1000

/**
@author Tom Krajnik
*/

using namespace std;

class CFrelement2DGridSet
{
 public:
  CFrelement2DGridSet();
  ~CFrelement2DGridSet();

  /*add new measurements
    - if the name (ID) is new, then a new state is created the function
    - if not, the measurements are added to the state with the given ID
    - returns the the index of the set in the collection*/
  /*添加新的测量值
    - 如果名称（ID）是新的，则创建该函数的新状态
    - 如果不是，则将测量值添加到具有给定ID的状态
    - 返回集合中集合的索引*/
  int add(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map);

  /*estimate occupancy of the cells for given times
    returns false if the state with the given ID is not present in the collection
    otherwise returns true and fills the map with the calculated predictions*/
  /*估计在给定时间的单元格的占用率
    如果集合中不存在具有给定ID的状态，则返回 false
    否则返回 true，并用计算的预测填充地图*/
  int estimate(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order);

  /*estimate occupancy entropy of the cells for given times 
    returns false if the state with the given ID is not present in the collection
    otherwise returns true and fills the map with the calculated predictions*/
  int estimateEntropy(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order);

  /*evaluate the prediction/estimation for the given times
    returns -1 if the state with the given ID is not present in the collection
    otherwise returns the best performing model order and the errors in the eval array*/
  /*评估给定时间的预测/估计
    如果集合中不存在具有给定ID的状态，则返回-1
    否则返回表现最佳的模型阶数和 eval 数组中的 errors*/
  int evaluate(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order,float errs[]);

  /*remove states from the collection
    return the number of remaining states*/
  int remove(const char *name);
  int save(const char *name,const char* filename);
  int load(const char *name,const char* filename);

  int print();

  CFrelement2DGrid* active;

 private:
  bool find(const char *name);

  // grids 为指针数组，它有 MAX_LENGTH 个指针类型的数组元素，每一个元素指向 CFrelement2DGrid 类型的变量
  CFrelement2DGrid* grids[MAX_LENGTH];
  int numGrids;
  int activeIndex;
};

#endif //CEDGESTATISTICS_H
