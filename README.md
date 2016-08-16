# RobotSimulatorGUI
This project demonstrate the NTU robot arm-hand simulator.
## Platform
Visual Studio 2013 - 64 bits 建置
## Libraries
Eigen 3.2.8<br> 
FreeGLUT 3.0.0<br> 
Bullet2 2.83.7<br> 
QHull 2015.2<br> 
PCL 1.7.2<br>
相關資料可於這邊查閱 https://goo.gl/KvFm3p
## 環境變數
如果是安裝上方網站所下載下來的 Prebuild Library<br>
此專案有附 property sheet 所以Link部分應該不用擔心 <br>
請在環境變數增加以下名稱<br>
EIGEN => Eigen主目錄絕對路徑<br>
FREEGLUT => FreeGLUT主目錄絕對路徑<br>
BULLET => Bullet主目錄絕對路徑<br>
QHULL => QHull主目錄絕對路徑<br>
PCL_ROOT => PCL主目錄絕對路徑<br>
PATH 增加 <br>%FREEGLUT%\lib\x64\Debug;<br>%FREEGLUT%\lib\x64\Release;<br>%PCL_ROOT%\bin;<br>%PCL_ROOT%\3rdParty\VTK\bin;<br>
因為PCL, FreeGLUT都是動態函式庫
## Robot Model
這部分攸關實驗室保密協定<br>
請去畢業光碟找手臂手掌CAD model<br>
