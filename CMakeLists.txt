# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

set(_idf_path "$ENV{IDF_PATH}")
if(NOT _idf_path)
	if(DEFINED IDF_PATH)
		set(_idf_path "${IDF_PATH}")
	else()
		file(GLOB _idf_candidates "$ENV{HOME}/.espressif/v*/esp-idf")
		list(SORT _idf_candidates ORDER DESCENDING)
		list(LENGTH _idf_candidates _idf_candidates_len)
		if(_idf_candidates_len GREATER 0)
			list(GET _idf_candidates 0 _idf_path)
			set(ENV{IDF_PATH} "${_idf_path}")
		endif()
	endif()
endif()

if(NOT _idf_path)
	message(FATAL_ERROR "ESP-IDF path not found. Set IDF_PATH or open this project via ESP-IDF extension.")
endif()

include(${_idf_path}/tools/cmake/project.cmake)
project(voda-septik)
