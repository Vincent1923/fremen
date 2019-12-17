#include "CFrelement2DGridSet.h"

using namespace std;

// 构造函数
CFrelement2DGridSet::CFrelement2DGridSet()
{
  numGrids = 0;
  activeIndex = 0;
  active = NULL;
}

// 析构函数
CFrelement2DGridSet::~CFrelement2DGridSet()
{
  for (int i = 0; i < numGrids; i++)
  {
	delete grids[i];  // 清空 Frelement 地图数据
  }
}

// 在时间 time 把新构建的地图 map 添加到 Frelement 地图集合中，Frelement 地图的名字为 name。
// 如果 name（ID）是新的，则在 Frelement 地图集合中创建新地图；
// 如果不是，则将测量值 map 添加到具有给定 ID 的 Frelement 地图中。
int CFrelement2DGridSet::add(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map)
{
  // find() 函数根据“name”查找 Frelement 地图集合 grids[i] 中是否已经存在对应名字的地图
  bool exists = find(name);
  if (exists == false)  // 如果找不到，则在 grids[] 数组中新建一张名字为 name 的 Frelement 地图
  {
	grids[numGrids++] = new CFrelement2DGrid(name);  // 新建 ID 为 name 的 Frelement 地图
	activeIndex = numGrids - 1;    // activeIndex 设置为当前新建的地图索引
	active = grids[numGrids - 1];  // active 指向当前新建的 Frelement 地图
  }

  // 在当前激活地图 active 中添加新构建的地图 map
  return active->add(time, map->data.data(), map->info.width, map->info.height,
                     map->info.origin.position.x, map->info.origin.position.y, map->info.resolution);
}

// find() 函数根据“name”查找 Frelement 地图集合 grids[i] 中是否已经存在对应 ID 的地图。
// 若没找到，则返回 false；若找到了，则返回 true，并且把 activeIndex 和 active 设置为与 name 名字对应的地图。
bool CFrelement2DGridSet::find(const char *name)
{
  int i = 0;

  for (i = 0; (i < numGrids) && (strcmp(grids[i]->id, name) != 0); i++) {}

  // 在 grid[i] 中找不到名字为“name”的 Frelement 地图
  if (i == numGrids)
  {
    return false;
  }

  // 在 grid[i] 中可以找到名字为“name”的 Frelement 地图
  activeIndex = i;
  active = grids[i];

  return true;
}

// 对栅格地图在时刻 time 的单元格占用率进行预测和估计，即在时刻 time 对可能出现的整张地图进行预测
int CFrelement2DGridSet::estimate(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map, int order)
{
  // 集合中不存在具有给定 ID 的地图，返回 -1
  if (find(name) == false)
  {
	return -1;
  }

  // 集合中存在具有给定 ID 的地图并且对地图预测成功，返回 true
  return active->estimate(time, map->data.data(), order);
}

int CFrelement2DGridSet::estimateEntropy(const char *name,uint32_t time,nav_msgs::OccupancyGrid *map,int order)
{
	if (find(name) == false)return -1;
	return active->estimateEntropy(time,map->data.data(),order);
}

// 评估给定时间的预测/估计
// 根据新构建的地图 map，在给定时刻 time 对 ID 为 name 的 Frelement 地图进行预测/评估，
// 主要是计算地图 map 与 Frelement 地图之间的误差，即计算新构建的地图与 Frelement 地图的匹配程度（一致性）。
int CFrelement2DGridSet::evaluate(const char *name, uint32_t time, nav_msgs::OccupancyGrid *map, int order, float errors[])
{
  // 集合中不存在具有给定 ID 的地图，返回 -1
  if (find(name) == false)
  {
    return -1;
  }

  // 计算地图一致性成功，返回表现最佳的模型阶数和 eval 数组中的 errors
  return active->evaluate(time, map->data.data(), order, errors);
}

int CFrelement2DGridSet::print()
{
  for (int i = 0; i < numGrids; i++)
  {
    printf("Map %s is %ix%i\n", grids[i]->id, grids[i]->width, grids[i]->height);
  }
}

int CFrelement2DGridSet::remove(const char *name)
{
  if (find(name) == false)
  {
    return -numGrids;
  }

  delete grids[activeIndex];

  grids[activeIndex] = grids[--numGrids];

  return numGrids + 1;
}

int CFrelement2DGridSet::save(const char *name, const char *filename)
{
  if (find(name) == false)
  {
    return -1;
  }

  active->save(filename);

  return numGrids;
}

int CFrelement2DGridSet::load(const char *name, const char *filename)
{
  bool exists = find(name);

  if (exists == false)
  {
    grids[numGrids++] = new CFrelement2DGrid(name);
    activeIndex = numGrids - 1;
    active = grids[numGrids - 1];
  }

  active->load(filename);

  return numGrids;
}
