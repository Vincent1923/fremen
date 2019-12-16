#include <stdlib.h>
#include "ros/ros.h"
#include "CFrelement.h"
#include "CFrelement2DGridSet.h"
#include <actionlib/server/simple_action_server.h>
#include <fremen2dgrid/Fremen2DGridAction.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

using namespace std;

float *periods = NULL;
ros::NodeHandle *n;

typedef actionlib::SimpleActionServer<fremen2dgrid::Fremen2DGridAction> Server;
Server* server;

fremen2dgrid::Fremen2DGridResult result;
fremen2dgrid::Fremen2DGridFeedback feedback;

bool debug = false;
uint32_t testTime = 0;
CFrelement *frelementArray = NULL;
float *values = NULL;
uint32_t times[1];
bool stop = false;
CFrelement2DGrid *gridA;
CFrelement2DGrid *gridB;
ros::Publisher pubMap;
int predictOrder = 2;

CFrelement2DGridSet* grids;
nav_msgs::OccupancyGrid predictedMap;
int numArrays = 0;
char mapName[100];

void actionServerCallback(const fremen2dgrid::Fremen2DGridGoalConstPtr& goal, Server* as)
{
  std::stringstream mess;

  //check if the element id is not too long
  // 检查输入的阶数 goal->order 是否符合要求，即是否在 0 到 NUM_PERIODICITIES 的范围内
  if (goal->order < 0 || goal->order > NUM_PERIODICITIES)
  {
    result.success = false;
    result.message = "Model order is out of bounds high. Reasonable value is 2, maximum is NUM_PERIODICITIES, minimum 0.";
    server->setAborted(result);
    return;
  }
  // if (debug) ROS_DEBUG("Command received %s %s\n",goal->operation.c_str(),goal->id.c_str());

  times[0] = goal->time;  // 'predict', 'add', 'entropy' and 'evaluate' actions 输入的时间戳
  nav_msgs::OccupancyGrid *mapPtr = (nav_msgs::OccupancyGrid*)&goal->map;  // mapPtr 指向输入的栅格地图

  if (goal->operation == "add")  // 在时刻 goal->time 加入栅格地图数据
  {
    // 加入栅格地图 mapPtr，并返回结果 resultCode
    int resultCode = grids->add(goal->mapName.c_str(), goal->time, mapPtr);

    if (resultCode == 1)
    {
      // Frelement 地图集合还不存在名字为 goal->mapName 的地图，新地图加入 Frelement 地图的集合中，返回加入地图成功结果
      mess << "New map " << goal->mapName << " was added to the map collection.";
      result.message = mess.str(); 
      server->setSucceeded(result);
    }
    else if (resultCode == 0)
    {
      // Frelement 地图集合已经存在名字为 goal->mapName 的地图，新构建的局部地图加入对应的 Frelement 地图中，并且进行地图更新，返回加入地图成功。
      mess << "Map " << goal->mapName << " was updated with measurement from time " << goal->time << ".";
      result.message = mess.str();
      server->setSucceeded(result);
    }
    else if (resultCode == -1)
    {
      // Frelement 地图集合已经存在名字为 goal->mapName 的地图，但是新构建的局部地图与 Frelement 地图的分辨率或者size（width, height）不同
      // 返回加入地图失败
      mess << "Map " << goal->mapName << " has different resolution or dimensions that the one you want to add.";
      result.message = mess.str();
      server->setAborted(result);
    }
  }
  else if (goal->operation == "predict")  // 在时刻 goal->time 对 Frelement 地图集合中名字为 goal->mapName 的地图进行预测
  {
    // 在时刻 goal->time 对地图进行预测，预测的地图保存在 mapPtr，并返回结果 resultCode
    int resultCode = grids->estimate(goal->mapName.c_str(), goal->time, mapPtr, goal->order);

    if (resultCode >= 0)
    {
      // 地图预测成功
      mess << "Predicted the state of " << goal->mapName << " for time " << goal->time << " using FreMEn order " << goal->order;
      result.message = mess.str();
      server->setSucceeded(result);
    }
    else  // resultCode < 0
    {
      // 地图预测失败，原因可能是 Frelement 地图集合中不存在名字为 goal->mapName 的地图
      mess << "Map " << goal->mapName << " was not in the map collection ";
      result.message = mess.str();
      server->setAborted(result);
    }
  }
  else if (goal->operation == "entropy")
  {
    int resultCode = grids->estimateEntropy(goal->mapName.c_str(), goal->time, mapPtr, goal->order);

    if (resultCode >= 0)
    {
      mess << "Predicted the uncertainty (entropy) of " << goal->mapName << " for time " << goal->time << " using FreMEn order " << goal->order;
      result.message = mess.str();
      server->setSucceeded(result);
    }
    else
    {
      mess << "Map " << goal->mapName << " was not in the map collection ";
      result.message = mess.str();
      server->setAborted(result);
    }
  }
  else if (goal->operation == "evaluate")
  {
	  // 根据新的局部地图 mapPtr，在给定时刻 goal->time 对 Frelement 地图集合中名字为 goal->mapName 的地图进行预测/评估，
	  // 主要是计算地图 mapPtr 与 Frelement 地图之间的误差，即计算局部地图与 Frelement 地图的匹配程度。
	  float errors[goal->order + 1];  // 地图误差

    // 评估在给定时间 goal->time 的预测/估计。
    // 若返回结果 resultCode >= 0，则评估成功，返回性能最佳的模型阶数以及 eval arra 中的误差；
	  // 若返回结果 resultCode = -1，则评估失败，原因可能是 Frelement 地图集合中不存在名字为 goal->mapName 的地图。
	  // errors[0] 的数值越小，地图的匹配程度越高。
	  int resultCode = grids->evaluate(goal->mapName.c_str(), goal->time, mapPtr, goal->order, errors);

	  if (resultCode >= 0)
	  {
	    // 评估成功
	    mess << "Evaluated the predictability of " << goal->mapName << " for time " << goal->time << " using FreMEn orders 0 to " << goal->order;
	    result.message = mess.str();
	    server->setSucceeded(result);
	  }
	  else
	  {
      // 评估失败，原因可能是 Frelement 地图集合中不存在名字为 goal->mapName 的地图
	    mess << "Map " << goal->mapName << " was not in the map collection ";
	    result.message = mess.str();
	    server->setAborted(result);
	  }
  }
  else if (goal->operation == "print")
  {
	  grids->print();
	  result.success = true;
	  result.message = "Debug printed";
	  server->setSucceeded(result);
  }
  else
  {
	  result.success = false;		
	  result.message = "Unknown action requested";
	  server->setAborted(result);
  }
}

int test()
{
	periods = (float*)calloc(NUM_PERIODICITIES,sizeof(float));
	for (int i=0;i<NUM_PERIODICITIES;i++) periods[i] = (24*3600)/(i+1); 
	gridA = new CFrelement2DGrid("A");
	gridB = new CFrelement2DGrid("B");
	//int8_t states[] = {30,0,100,-1, -1,-1,-1,100, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};
	int8_t *states = (int8_t*) calloc(1000000,1);

	printf("Add0\n");
	for (int i = 0;i<1000000;i++) states[i] = 0;
	int dim = 1000;
	gridA->add(0,states,dim,dim,0,0,1);

	printf("Add1\n");
	for (int i = 0;i<1000000;i++) states[i] = 100;
	gridA->add(3600,states,dim,dim,0,0,1);

	printf("Add2\n");
	for (int i = 0;i<1000000;i++) states[i] = 0;
	gridA->add(7200,states,dim,dim,0,0,1);

	printf("Add3\n");
	for (int i = 0;i<1000000;i++) states[i] = 100;
	gridA->add(10800,states,dim,dim,0,0,1);
	//gridA->print(1);

	gridA->estimate(1900,states,1);
	printf("Estimate\n");
	for (int i = 0;i<1;i+=1) printf("%i\n",states[i]);

	printf("Save\n");
	gridA->save("test.grid");

	gridB->load("test.grid");
	gridB->estimate(1900,states,1);
	for (int i = 0;i<1;i+=1) printf("%i\n",states[i]);
	//gridB->print(0);
	free(periods);
	delete gridA;
	delete gridB;
	free(states);
}

void mapCallback(const nav_msgs::OccupancyGrid &msg)
{
	int8_t *data = (int8_t*) msg.data.data();

  // 把订阅的地图 msg 加入 Frelement 地图的集合中，名字为 msg.header.frame_id，加入时间为 msg.info.map_load_time.sec。
  // 这里没有考虑地图 msg 与 Frelement 地图的一致性，即匹配程度。
	// grids->add(msg.header.frame_id.c_str(), msg.info.map_load_time.sec, (nav_msgs::OccupancyGrid*)&msg);

	float errors[5];  // 地图误差

  // 评估地图 msg 与 Frelement 地图的一致性，并返回最佳的模型阶数 bestO 以及误差 errors。
  // 输入阶数为4，Frelement 地图的名字为 mapName，评估的时间为 testTime。
	int bestO = grids->evaluate(mapName, testTime, (nav_msgs::OccupancyGrid*)&msg, 4, errors);
	printf("Best model %i %f\n",bestO,errors[bestO]);

  // 根据地图 msg 与 Frelement 地图一致性的评估结果，判断是否把订阅的地图 msg 加入 Frelement 地图的集合中，
  // 并获取预测地图 predictedMap。
	if (errors[bestO] < 0.025)  //0.03 without recency
  {
    // 把订阅的地图 msg 加入 Frelement 地图的集合中，名字为 mapName，加入时间为 testTime。
		int result = grids->add(mapName, testTime, (nav_msgs::OccupancyGrid*)&msg);

		if (result ==  0)  // Frelement 地图集合中已存在名字为 mapName 的地图，加入新地图 msg 并对 Frelement 地图进行更新
    {
      printf("Map %s updated with info from %i\n", mapName, testTime);
    }

		if (result ==  1)  // Frelement 地图集合中还不存在名字为 mapName 的地图，把新地图 msg 加入 Frelement 地图集合
    {
      printf("New map %s created from time %i\n", mapName, testTime);
    }
    
		if (result == -1)  // 新地图 msg 与 Frelement 地图的分辨率或者 size 不同
    {
      printf("Map %s dimension mismatch\n", mapName);
    }
    
		if (result >= 0)
    {
      predictedMap = msg;
    }
    
    // 对更新的地图进行预测，预测的时间为 testTime，预测的地图保存在 predictedMap，阶数为0
		grids->estimate(mapName, testTime, &predictedMap, 0);
	}

  // 发布预测地图 predictedMap
	pubMap.publish(predictedMap);
}

void nameCallback(const std_msgs::String &msg)
{
	strcpy(mapName,msg.data.c_str());
}

void predictTimeCallback(const std_msgs::Int64 &msg)
{
	grids->estimate(mapName,(int32_t)msg.data,&predictedMap,predictOrder);
	pubMap.publish(predictedMap);
}

void predictOrderCallback(const std_msgs::Int64 &msg)
{
	predictOrder = (int)msg.data;
}

void mapSaveCallback(const std_msgs::String &msg)
{
	grids->save(mapName,msg.data.c_str());
}

void addTimeCallback(const std_msgs::Int64 &msg)
{
	testTime = (int32_t)msg.data;
}

int main(int argc,char* argv[])
{
  ros::init(argc, argv, "fremen2dgrid");

  n = new ros::NodeHandle();

  // 初始化 actionlib 服务器
  server = new Server(*n, "/fremenarray", boost::bind(&actionServerCallback, _1, server), false);
  server->start();

  /*
   * （1）periods 表示地图更新中傅里叶变换的周期，这是一个长度为 NUM_PERIODICITIES，每个元素为 float 类型的一维数组。
   * （2）函数原型：void* calloc (size_t num, size_t size);
   *     函数功能：在内存的动态存储区中分配 num 个长度为 size 的连续空间，函数返回一个指向分配起始地址的指针；如果分配不成功，返回 NULL。
   *             为一个元素个数为 num 的数组分配一个内存块，每个元素的大小为 size 字节的长度，并将其所有 bits 位初始化为零。
   *             有效的结果是分配了一个字节数为(num * size)，初始化为零（zero-initialized）的存储块。
   *             如果 size 为零，则返回值取决于特定的库实现（它可以是空指针，也可以不是空指针），但是不得取消对返回的指针的引用。
   *     返回值：成功时，指向函数分配的内存块的指针。该指针的类型始终为 void*，可以将其强制类型转换为所需的数据指针类型，以便将其取消引用。
   *            如果函数未能分配所请求的内存块，则返回空指针 NULL。
   *            （void* 表示未确定类型的指针，void* 可以指向任何类型的数据，更明确的说是指申请内存空间时
   *             还不知道用户是用这段空间来存储什么类型的数据（比如是 char 还是 int 或者其他数据类型）。）
   *  （3）与 malloc 的区别：calloc 在动态分配完内存后，自动初始化该内存空间为零，而 malloc 不做初始化，分配到的空间中的数据是随机数据。
   */
  periods = (float*)calloc(NUM_PERIODICITIES, sizeof(float));  // (float*) 表示强制类型转换，periods 指向的数组的数据类型为 float
  for (int i = 0; i < NUM_PERIODICITIES; i++)  // NUM_PERIODICITIES 表示周期的数量，默认值为24
  {
	periods[i] = (24 * 3600) / (i + 1);  // 周期为 24h,12h...1h
  }

  grids = new CFrelement2DGridSet();  // Frelement 地图的集合

  // mapName 为 Frelement 地图的名字，这里初始化为"default"
  strcpy(mapName, "default");
  if (argc > 1) grids->load(mapName,argv[1]);

  // Subscriber
  ros::Subscriber subMap = n->subscribe("/map", 1, mapCallback);
  ros::Subscriber subPredTime = n->subscribe("/predictTime", 1, predictTimeCallback);
  ros::Subscriber subPredOrder = n->subscribe("/predictOrder", 1, predictOrderCallback);
  ros::Subscriber subAddTime = n->subscribe("/addTime", 1, addTimeCallback);
  ros::Subscriber subName = n->subscribe("/mapName", 1, nameCallback);
  ros::Subscriber subSAve = n->subscribe("/mapSave", 1, mapSaveCallback);

  // Publisher
  pubMap = n->advertise<nav_msgs::OccupancyGrid>("/predictedMap", 1);

  while (ros::ok())
  {
	ros::spinOnce();
	usleep(30000);
  }

  ros::shutdown();

  delete server;
  delete n;

  return 0;
}
