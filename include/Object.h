#ifndef OBJECT_H
#define OBJECT_H

#include "MapPoint.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include <mutex>

namespace ORB_SLAM2{
class MapPoint;
class Map;

class Object{
public:
	Object(int& mObjectId, int&mClassId, MapPoint* mPoint, Map* oMap);
	Object(int& mObjectId, int&mClassId, Map* oMap);
	void AddMapPoint(MapPoint* mPoint);
	void EraseMapPoint(MapPoint* mPoint);
	int GetObjectId();
	int GetClassId();
	void SetObjectId(int& classId);
	void SetObjectPose(cv::Mat setPose);
	cv::Mat GetObjectPose();
	int GetSize();
	std::vector<MapPoint*> GetAllMapPoints();
	void ObjectMerge(Object* obj);
	void SetBadFlag(bool flag = true);
	bool isBad();
	bool IsChangeable();
	void ResetDuration();
protected:
	Map* objMap;
	bool objBad;
	int objectId;
	int classId;
	std::set<MapPoint*> oMapPoints;
	cv::Mat oPose;
	std::mutex mMutexObj;
	int duration;
};
}
#endif