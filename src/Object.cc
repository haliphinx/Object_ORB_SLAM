#include "Object.h"

namespace ORB_SLAM2{

	Object::Object(int& mObjectId, int& mClassId, MapPoint* mPoint, Map* oMap){
		objectId = mObjectId;
		classId = mClassId;
		oMapPoints.insert(mPoint);
		oPose = cv::Mat::zeros(4,4,CV_8UC1);
		objBad = false;
		objMap = oMap;
		duration = 2;
	}
	Object::Object(int& mObjectId, int& mClassId, Map* oMap){
		objectId = mObjectId;
		classId = mClassId;
		oPose = cv::Mat::zeros(4,4,CV_8UC1);
		objBad = false;
		objMap = oMap;
		duration = 2;
	}
	void Object::AddMapPoint(MapPoint* mPoint){
		unique_lock<mutex> lock(mMutexObj);
		oMapPoints.insert(mPoint);
	}
	void Object::EraseMapPoint(MapPoint* mPoint){
		unique_lock<mutex> lock(mMutexObj);
		oMapPoints.erase(mPoint);
	}
	int Object::GetObjectId(){
		unique_lock<mutex> lock(mMutexObj);
		return objectId;
	}
	int Object::GetClassId(){
		unique_lock<mutex> lock(mMutexObj);
		return classId;
	}
	void Object::SetObjectId(int& classId){
		unique_lock<mutex> lock(mMutexObj);
		objectId = classId;
	}
	void Object::SetObjectPose(cv::Mat setPose){
		unique_lock<mutex> lock(mMutexObj);
		oPose = setPose;
	}
	int Object::GetSize(){
		unique_lock<mutex> lock(mMutexObj);
		return oMapPoints.size();
	}
	cv::Mat Object::GetObjectPose(){
		unique_lock<mutex> lock(mMutexObj);
		return oPose;
	}
	std::vector<MapPoint*> Object::GetAllMapPoints(){
		unique_lock<mutex> lock(mMutexObj);
		return vector<MapPoint*>(oMapPoints.begin(),oMapPoints.end());
	}
	void Object::ObjectMerge(Object* obj){
		std::vector<MapPoint*> objMp = obj->GetAllMapPoints();
		for(size_t i = 0; i<objMp.size(); i++){
			int newId = this->GetObjectId();
			objMp[i]->SetMapPointId(newId, true);
		}
	}
	void Object::SetBadFlag(bool flag){
		objBad = flag;
		// if(flag){
		// 	oMapPoints.clear();
		// 	objMap->EraseObject(this);
		// }
	}
	bool Object::isBad(){
		return objBad;
	}
	bool Object::IsChangeable(){
		if(duration == 0){
			return true;
		}
		duration--;
		return false;
	}
	void Object::ResetDuration(){
		duration = 2;
	}
}// namespace ORB_SLAM2