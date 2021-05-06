## Cutomzing plugin to make depth frame as mesh in real time and work on gazebo

Order

1. Get depth frame from realsense. I'm trying to publish depth frame and subscribe to it in the plugin. 

2. In plugin file, convert the depth frame to mesh. Mesh data should be made in the form of a dae file.

3. There's two way to show mesh shape on gazebo. One is inserting dae file name on gazebo using insertmodelfile API, and the another is inserting dae code 
directly in gazebo source code. First method is quiet easy, but second method could be hard for you that there's no API that gazebo supports.

That I have to convert depth frame to mesh in real time, I should not use first method. It's too long to wait making dae file and inserting it.
I will use second method but there's no API that I was to use, so I have to make my own plugin.

You should understand how the gazebo world update models and states. You can analyze gazebo open source code in url below.
(https://github.com/osrf/gazebo/tree/2e42eb91e7638da54b36dbb8fdd4ae5e3a310d6d)


As you can see in source code, all of variables are modified while threads are locked(means blocking the calling thread). Then, when you execute notify function,
all of the threads will be called and world will be updated. Did you understand what you have to do? Right. When you complete converting depth frame to mesh,
lock threads, insert the mesh collada data into function related to mesh, and notify.

See MeshShape, MeshManager, ColladaLoad file to analyze source code related to mesh.



