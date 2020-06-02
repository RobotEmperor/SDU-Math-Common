CMAKE_MINIMUM_REQUIRED ( VERSION 2.8 )
 
PROJECT ( "sdu_math" )
SET ( PROJECT_VERSION_MAJOR 0 )
SET ( PROJECT_VERSION_MINOR 1 )

SET ( CMAKE_C_COMPILER "gcc" )
 
INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/sdu_math/include)

add_compile_options(-std=c++11)
 
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

set(LIB_HEADER_FILES
   ${CMAKE_SOURCE_DIR}/sdu_math/include/sdu_math/control_function.h
   ${CMAKE_SOURCE_DIR}/sdu_math/include/sdu_math/kinematics.h
   ${CMAKE_SOURCE_DIR}/sdu_math/include/sdu_math/statistics_math.h
   ${CMAKE_SOURCE_DIR}/sdu_math/include/sdu_math/end_point_to_rad_cal.h
   ${CMAKE_SOURCE_DIR}/sdu_math/include/sdu_math/fifth_order_trajectory_generate.h
   )

set( sdu_math_SOURCE_FILES 
 sdu_math/src/control_function.cpp
 sdu_math/src/kinematics.cpp
 sdu_math/src/statistics_math.cpp
 sdu_math/src/end_point_to_rad_cal.cpp
 sdu_math/src/fifth_order_trajectory_generate.cpp
)


add_library(sdu_math SHARED ${sdu_math_SOURCE_FILES} ${LIB_HEADER_FILES})


if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
		set_target_properties(sdu_math PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	endif()

	if(NOT DEFINED LIB_INSTALL_DIR)
		set(LIB_INSTALL_DIR "lib")
	endif()
	if(NOT DEFINED BIN_INSTALL_DIR)
		set(BIN_INSTALL_DIR "bin")
	endif()
	if(NOT DEFINED INCLUDE_INSTALL_DIR)
		set(INCLUDE_INSTALL_DIR "include")
	else()
		set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}")
	endif()

install(TARGETS sdu_math
			LIBRARY DESTINATION ${LIB_INSTALL_DIR}
			ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
			RUNTIME DESTINATION ${BIN_INSTALL_DIR}
			INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
			)
install(
			FILES
			${LIB_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/sdu_math
			COMPONENT sdu_math
	)

 
 
