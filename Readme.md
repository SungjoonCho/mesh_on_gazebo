## Cutomzing plugin to make depth frame as mesh in real time and work on gazebo

Order

1. Get depth frame from realsense. I'm trying to publish depth frame and subscribe to it in the plugin. 

2. In plugin file, convert the depth frame to mesh. Mesh data should be made in the form of a dae file.

3. There's two way to show mesh shape on gazebo. One is inserting dae file name on gazebo using insertmodelfile API, and the another is inserting dae code 
directly in gazebo source code. First method is quiet easy, but second method could be hard for you that there's no API that gazebo supports.

That I have to convert depth frame to mesh in real time, I should not use first method. It's too long to wait making dae file and inserting it.
I will use second method but there's no API that I can use, so I have to make my own plugin.

You should understand how the gazebo world update models and states. You can analyze gazebo open source code in url below.
(https://github.com/osrf/gazebo/tree/2e42eb91e7638da54b36dbb8fdd4ae5e3a310d6d)


As you can see in source code, all of variables are modified while threads are locked(means blocking the calling thread). Then, when you execute notify function,
all of the threads will be called and world will be updated. Did you understand what you have to do? Right. When you complete converting depth frame to mesh,
lock threads, insert the mesh collada data into function related to mesh, and notify.

See MeshShape, MeshManager, ColladaLoad file to analyze source code related to mesh.





## command
<pre>
terminal0 : roscore

terminal1(making plugin) : In build directory
~/gazebo_plugin_pc/build$ make

terminal2(running world) : In MyWorld directory
~/gazebo_plugin_pc/MyWorld$ gazebo --verbose realsense_depth_to_mesh.world
killall gzserver
</pre>

## 5.17

1. ~/model/info topic으로 publish 하는 데이터를 subscribe 받은 이후 sdf 파일 내 mesh uri 파일 데이터를 파싱하는 방식
<pre>
문제점 : sdf 파일을 굳이 개입시키지 않고(실시간으로 파일을 별도로 만들고 로드하는데 시간이 너무 오래 걸림) mesh data만 가져오고자 함
</pre>

2. ColladaLoader에서 진행하는 바와 같이 공유변수 수정하는 방식
<pre>
문제점

마지막 mesh 파싱 과정-> ColladaLoader를 이용해 생성된 객체 Dataptr은 ColladaLoaderPrivate이며 
이와 동일한 방식으로 Customizing 하는 플러그인에서 접근해야 함  
ColladaLoaderPrivate은 내가 수정 중인 플러그인에서 접근불가(Private 파일은 이미 빌드된 채로 설치됨) 
그래서 sdf + mesh 파싱하는 전체 디렉토리를 새로 구성해야 되서 많은 시간 소요
</pre>

3. Model file의 ModelToSDF 이용
<pre>
a. model, geometry 생성

b. meshgeom 새로 생성하여 collada file(box dae sameple file)의 data(tag, text)를 직접 넣고 미리 만든 geometry, model과 
연결하여 msg 생성
</pre>

-결과 

```html
<model>
	<link>
		<geometry>
			<mesh>ColladaData</mesh>
		</geometry>
	</link>
	<visual>
		<geometry>
			<mesh>ColladaData</mesh>
		</geometry>
	</visual>
</model>
```

<pre>
c. 만들어진 msg를 ModelToSDF 하여 완전한 sdf 형식으로 변경 후 InsertModelString or ~/factory topic으로 publish


-InsertModelString 방식 문제점
string을 factoryMsgs에 pushback하며 이어서 sdf+mesh를 파싱해야(mesh tage에서 filename을 찾음) 하는 1번에서 겪은 문제와 동일한 문제 발생

-factoryPub에 넣은 이후 Publish하는 방식 문제점
위와 마찬가지로 factoryMsgs에 pushback하여 동일한 문제 발생
</pre>

<pre>
시작과 과정을 어떻게 하든 결국은 Colladaloader에서 mesh filename을 찾아 file의 data를 가져오는 방식으로 감.
하지만 이는 real time으로 mesh를 제작해 gazebo에 띄워야 하기에는 적합 하지 않으며  
이 외의 방식으로는 sdf를 파싱하는 구조, 디렉토리 전체를 수정해야 함. 다른 방법 좀 더 찾아보기
</pre>

