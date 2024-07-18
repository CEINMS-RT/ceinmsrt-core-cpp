Requirements:
MTUSpline compilled in Release

- Run Matlab as Administrator
- edit classpath.txt -> add at the end:

C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\MTUSplineAll.jar
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release\MTUSplineInterfaceJava.dll
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release\MTUSplineInterfaceJava.exp
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release\MTUSplineInterfaceJava.lib
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\

- pathtool -> add:

C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release

- edit librarypath.txt -> add at the end:

C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release
C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win\Release\MTUSplineInterfaceJava.dll

- Restart Matlab
- go to C:\Users\DurandauGV\Documents\CEINMS\CEINMS-RT\trunk\bin\Win
- import MTUSplineInterfaceJava.*
- LoadLibrary.loadLibrary('MTUSplineInterfaceJava');
 
Test:

>> a = vectors()

a =

MTUSplineInterfaceJava.vectors@13e3c1c7

>> a.add('a')

>> a.get(0)

ans =

a

