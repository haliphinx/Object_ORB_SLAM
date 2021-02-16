# Object_ORB_SLAM2
Fork from [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2), added the object detection, the object based 2D-3D point macthing and object based map point creation, so that the object can be generate from 2D image to 3D points cloud.

# 1. Object Detection
Use [Mask-RCNN](https://github.com/matterport/Mask_RCNN) to detect objects for each keyframes.

# 2. 2D Keypoint Generation
Give each keypoint a new attribute named object Id, which is connected to the object it belongs to.
![obj1](/res/obj1.png)
![obj2](/res/obj2.png)

# 3. 3D Map Point Cloud Generation
For each object detected by Mask-RCNN in each keyframe, generateing the 3D map points based on 2D keypoint with object Id. So that the 3D points in the map are no longer the meanless sparse point, but a part of object point cloud. After we get the point cloud for object, we can achieve more interactions with objects in AI(like AR, Auto Driving).
![map](/res/mappoint.png)
We can see in the image above, red points represent for monitor, green points stand for keyboard, blue points stand for plant and yellow points stanfd for desk.
