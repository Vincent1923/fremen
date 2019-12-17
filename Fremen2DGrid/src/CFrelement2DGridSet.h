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
  /**
   * @brief add   添加新的测量值，即添加新构建的地图到 Frelement 地图集合中
   * @param name  Frelement 地图的 ID
   * @param time  添加新构建地图的时间
   * @param map   新的测量值，即新构建的地图
   * @return      集合中 ID 为 name 的地图索引
   */
  int add(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map);

  /*estimate occupancy of the cells for given times
    returns false if the state with the given ID is not present in the collection
    otherwise returns true and fills the map with the calculated predictions*/
  /*估计在给定时间的单元格的占用率
    如果集合中不存在具有给定ID的状态，则返回 false
    否则返回 true，并用计算的预测填充地图*/
  /**
   * @brief estimate  在给定时间对地图单元格的占用率进行预测和估计，即在时刻 time 对可能出现的整张地图进行预测
   * @param name      Frelement 地图的 ID
   * @param time      对地图单元格的占用率进行预测和估计的时间
   * @param map       预测后的地图
   * @param order     输入的阶数
   * @return          如果 Frelement 地图集合中不存在具有给定 ID 的地图，则返回 false；否则返回 true，并用计算的预测填充地图
   */
  int estimate(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map, int order);

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

  CFrelement2DGrid* active;  // 指向当前激活的 Frelement 地图

 private:
  /**
   * @brief find  根据“name”查找 Frelement 地图集合 grids[i] 中是否已经存在对应 ID 的地图
   * @param name  需要查找的 ID
   * @return      若没找到，则返回 false；若找到了，则返回 true
   */
  bool find(const char *name);

  // grids 为 Frelement 地图数组
  // grids 是一个指针数组，它有 MAX_LENGTH 个指针类型的数组元素，每一个元素指向 CFrelement2DGrid 类型的变量
  CFrelement2DGrid* grids[MAX_LENGTH];
  int numGrids;     // Frelement 地图数组的地图数量
  int activeIndex;  // 当前激活的地图的下标
};

#endif //CEDGESTATISTICS_H
