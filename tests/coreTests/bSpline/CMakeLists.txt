ADD_EXECUTABLE(TestBoostBSpline
	BsplineTest.cpp
)


target_link_libraries(TestBoostBSpline
	MTUSpline
	FileIO
	ws2_32
	winmm
	${Boost_LIBRARIES}
	${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)



enable_testing()

add_test(NAME BsplineTest COMMAND $<TARGET_FILE:TestBoostBSpline>)
