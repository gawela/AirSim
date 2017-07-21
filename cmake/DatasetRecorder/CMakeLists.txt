cmake_minimum_required(VERSION 3.5.0)
project(DatasetRecorder)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

SetupConsoleBuild()

include_directories(
  ${AIRSIM_ROOT}/DatasetRecorder
  ${AIRSIM_ROOT}/DatasetRecorder/src
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/DatasetRecorder/include;
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
${AIRSIM_ROOT}/MavLinkCom/common_utils
)

AddExecutableSource()
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
