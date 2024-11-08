add_library(FileLogger 
        OpenSimFileLogger.h
        )

target_link_libraries(FileLogger PUBLIC
        NMSmodel
        FileIO   
        Boost::filesystem
        # Boost::system             
  )

target_include_directories(FileLogger PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}) 

add_library(ModelEvaluation
        ModelEvaluationBase.cpp
        ModelEvaluationRealTime.h
        )

target_link_libraries(ModelEvaluation PUBLIC
        FileLogger               
  )

  target_include_directories(FileLogger PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}) 