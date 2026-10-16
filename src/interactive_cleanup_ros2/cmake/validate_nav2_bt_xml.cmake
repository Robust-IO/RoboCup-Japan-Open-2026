file(READ "${INPUT_FILE}" NAV2_PARAMS)

string(REGEX MATCH "default_nav_to_pose_bt_xml:[ \t]*\"([^\"]*)\"" MATCHED_LINE "${NAV2_PARAMS}")

if(NOT MATCHED_LINE)
  message(FATAL_ERROR "default_nav_to_pose_bt_xml entry is missing from ${INPUT_FILE}")
endif()

set(BT_XML_PATH "${CMAKE_MATCH_1}")
string(STRIP "${BT_XML_PATH}" BT_XML_PATH)

if(BT_XML_PATH STREQUAL "")
  message(FATAL_ERROR "default_nav_to_pose_bt_xml must not be empty in ${INPUT_FILE}")
endif()
