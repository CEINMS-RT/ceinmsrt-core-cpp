IF(COMPILE_TWINCAT_PLUGIN OR COMPILE_TWINCAT_EMG_PLUGIN OR COMPILE_XSENS_PLUGIN)
	execute_process(
		COMMAND git submodule update --init plugin/lib/AdsClient
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
	add_subdirectory(lib/AdsClient)
ENDIF()

option(COMPILE_EMG_DELSYS_PLUGIN "Setting this parameter to ON will compile the EMG Delsys plugin for CEINMS." OFF)
IF(COMPILE_EMG_DELSYS_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/EMG_Delsys/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/EMG_Delsys
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule EMG_Delsys not initialized because folder EMG_Delsys folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(EMG_Delsys)
ENDIF()

option(COMPILE_EMG_UDP_SIMULINK_PLUGIN "Setting this parameter to ON will compile the EMG UDP Simulink plugin for CEINMS." OFF)
IF(COMPILE_EMG_UDP_SIMULINK_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/EMG_UDP_Simulink/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/EMG_UDP_Simulink
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule EMG_UDP_Simulink not initialized because folder EMG_UDP_Simulink folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it iniutialize.")
	endif()
	add_subdirectory(EMG_UDP_Simulink)
ENDIF()

option(COMPILE_ROS_PLUGIN "Setting this parameter to ON will compile the ROS plugin for CEINMS." OFF)
IF(COMPILE_ROS_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/ROSPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/ROSPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule ROSPlugin not initialized because folder ROSPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(ROSPlugin)
ENDIF()

option(COMPILE_INSOLE_PLUGIN "Setting this parameter to ON will compile the insole plugin for CEINMS." OFF)
IF(COMPILE_INSOLE_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/insolePlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init --recursive plugin/insolePlugin 
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule InsolePlugin not initialized because folder insolePlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(insolePlugin)
ENDIF()

option(COMPILE_RTOSIM_PLUGIN "Setting this parameter to ON will compile the RTOSIM plugin for CEINMS." OFF)
IF(COMPILE_RTOSIM_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/RTOSIMPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/RTOSIMPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule RTOSIMPlugin not initialized because folder RTOSIMPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(RTOSIMPlugin)
ENDIF()

option(COMPILE_TORQUE_POSITION_UDP_SIMULINK_PLUGIN "Setting this parameter to ON will compile the Torque Position UDP Simulink plugin for CEINMS." OFF)
IF(COMPILE_TORQUE_POSITION_UDP_SIMULINK_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/TorquePositionUDPPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/TorquePositionUDPPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule TorquePositionUDPPlugin not initialized because folder TorquePositionUDPPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(TorquePositionUDPPlugin)
ENDIF()

option(COMPILE_TWINCAT_PLUGIN "Setting this parameter to ON will compile the Twincat plugin for CEINMS." OFF)
IF(COMPILE_TWINCAT_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/TwincatPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/TwincatPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule TwincatPlugin not initialized because folder TwincatPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(TwincatPlugin)
ENDIF()

option(COMPILE_TWINCAT_EMG_PLUGIN "Setting this parameter to ON will compile the Twincat EMG plugin for CEINMS." OFF)
IF(COMPILE_TWINCAT_EMG_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/TwincatEMGPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/TwincatEMGPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule TwincatEMGPlugin not initialized because folder TwincatEMGPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(TwincatEMGPlugin)
ENDIF()

option(COMPILE_XSENS_PLUGIN "Setting this parameter to ON will compile the XSENS plugin for CEINMS." OFF)
IF(COMPILE_XSENS_PLUGIN)
	file(GLOB RESULT ${PROJECT_SOURCE_DIR}/plugin/XsensPlugin/*.txt) #this bit of code if for not erasing thew plugin if we are working on it. otherwise is call git submodules init and erase ther change
	if(RESULT STREQUAL "")
		execute_process(
			COMMAND git submodule update --init plugin/XsensPlugin
			WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		)
	else()
		message(WARNING "Submodule XsensPlugin not initialized because folder XsensPlugin folder already contains a .txt file (mostly a Cmakelist.txt). Empty the folder if you want it initialize.")
	endif()
	add_subdirectory(XsensPlugin)
ENDIF()











