Requirements:
MTUSpline need to be compilled in Debug

- in matlab Go to -> C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\Matlab

- Use CompileBSplineTestDebug.m to create a Mex of MTUSpline (make sure all path are OK)

- Launch simulink

- Start BsplineSim project to see an example

- In Code->C/C++ Code->Code Generations Option ...
	- System Target = grt.lc (visual studio projet (needed if problem))
	- Language C++
	- Custome Code
		- Include directories:

C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\MTUSpline
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\NMSmodel
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\Curve
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\ModelEvaluation
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\FileIO
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\OdeInt
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\lib\xmlInput
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\src
"C:\Program Files (x86)\CodeSynthesis XSD 4.0\include"
C:\local\boost_1_64_0
C:\local\boost_1_64_0\lib64-msvc-12.0
C:\local\boost_1_64_0
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Debug

		- libraries:

C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Debug\MTUSpline.lib
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Debug\xmlInput.lib
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Debug\NMSmodel.lib
"C:\Program Files (x86)\CodeSynthesis XSD 4.0\lib64\vc-12.0\xerces-c_3.lib"
C:\local\boost_1_64_0\lib64-msvc-12.0\libboost_timer-vc120-mt-1_64.lib
"C:\Program Files\MATLAB\R2016b\extern\lib\win64\microsoft\libmx.lib"
"C:\Program Files\MATLAB\R2016b\extern\lib\win64\microsoft\libmex.lib"

- Build Model


- Run

S-Function parameter:
	name: Bspline
	parameters: int8('cfg/TestData/gait2392Left.xml'),int8('TestData')
	modules: Bspline_wrapper

Input: positiopn (Max 32)
Outpput:Lmt (Max 32)
	MA (Max 32*32)

