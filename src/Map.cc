/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    // std::cout<<"as1"<<std::endl;
    mspKeyFrames.insert(pKF);
    // std::cout<<"as2"<<std::endl;
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }
    int pId = pMP->GetMapPointId();
    if(pId == -1){
        return;
    }
    
    Object* obj = GetObjectById(pId);
    if(obj){
        obj->AddMapPoint(pMP);
        // obj->ResetDuration();
    }
    else{std::cout<<"AddMapPoint"<<std::endl;}

}

void Map::AddObject(Object *pOB){
    unique_lock<mutex> lock(mMutexMap);
    mspObjects.insert(pOB);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    
    int curId = pMP->GetMapPointId();
    if(curId != -1){
       Object* curObject = GetObjectById(curId);
       if(curObject){
            curObject->EraseMapPoint(pMP); 
       }
       else{std::cout<<"EraseMapPoint"<<std::endl;}
       
    }
    
    mspMapPoints.erase(pMP);
    // delete pMP;
    // pMP = NULL;

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    // delete pKF;
    // pKF = NULL;

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseObject(Object *pOB){
        std::cout<<"push the object id: "<<pOB->GetObjectId()<<std::endl;
        nextObjectId.push_back(pOB->GetObjectId());
}

void Map::ObjectCulling(){
    // std::vector<Object*> allObj = GetAllObjects();
    {
        unique_lock<mutex> lock(mMutexMap);
        
        // std::cout<<"obj culling"<<std::endl;
        for(auto iter = mspObjects.begin();iter!=mspObjects.end();){
            // std::cout<<"obj culling1"<<std::endl;
            if((*iter)->isBad()){
                std::cout<<"Delete Object: "<<(*iter)->GetObjectId()<<std::endl;
                mspObjects.erase(iter++);
                  
            }
            else{
                iter++; 
            }
            
        }
    }
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

void Map::SetMapPointId(MapPoint* pMp, int& newId){
    Object* curObject = NULL;
    Object* nextObject = NULL;
    int curId;
    {
        unique_lock<mutex> lock(mMutexMap);
        curId = pMp->GetMapPointId();
        if(curId!=-1){
            curObject = GetObjectById(curId);
        }
        if(newId!=-1){
            nextObject = GetObjectById(newId);
        }
        
    }
    if(pMp->SetMapPointId(newId,false)){
        if(curObject){
            curObject->EraseMapPoint(pMp);
        }
        if(nextObject){
            nextObject->AddMapPoint(pMp);
        }
    }
    
    
    // else{std::cout<<"setmappointid"<<std::endl;}
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<Object*> Map::GetObjectByClassId(int& classId){
    std::vector<Object*> objectByClassId;
    std::vector<Object*> allObject = GetAllObjects();
    for (size_t i =0; i<allObject.size(); i++)
    {
        if(allObject[i]->GetClassId() == classId){
            objectByClassId.push_back(allObject[i]);
        }
    }
    allObject.clear();
    return objectByClassId;

}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<Object*> Map::GetAllObjects(){
    unique_lock<mutex> lock(mMutexMap);
    return vector<Object*>(mspObjects.begin(),mspObjects.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

 long unsigned int Map::ObjectsInMap(){
    unique_lock<mutex> lock(mMutexMap);
    return mspObjects.size();
 }

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

bool Map::IsObjectExist(int& classId){
    std::vector<Object*> obj = Map::GetAllObjects();
    for(int n = 0; n<int(obj.size()); n++){
        if(obj[n]->GetObjectId()==classId){
            return true;
        }
    }
    return false;
}

int Map::GetNextObjectId(){
    unique_lock<mutex> lock(mMutexMap);
    // std::cout<<"next object id."<<std::endl;
    if(nextObjectId.empty()){
        return mspObjects.size();
    }
    // std::cout<<"next id list is not empty."<<std::endl;
    nextObjectId.sort();
    int nextId = nextObjectId.front();
    std::cout<<"next object id is "<<nextId<<std::endl;
    nextObjectId.pop_front();
    // std::cout<<"next object pop"<<std::endl;
    return nextId;
}

int Map::GetNextObjSize(){
    unique_lock<mutex> lock(mMutexMap);
    return nextObjectId.size();
}


Object* Map::GetObjectById(int& classId){
    // unique_lock<mutex> lock(mMutexMap);
    vector<Object*> allObjects = vector<Object*>(mspObjects.begin(),mspObjects.end());
    for(size_t n = 0; n<allObjects.size(); n++){
        if((allObjects[n]->GetObjectId() == classId)&&(!allObjects[n]->isBad())){
            return allObjects[n];
        }
    }
    if(classId != -1){
       std::cout<<"Can't find object: "<<classId<<std::endl; 
    }
    
    return NULL;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    for(set<Object*>::iterator sit=mspObjects.begin(), send=mspObjects.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspObjects.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
