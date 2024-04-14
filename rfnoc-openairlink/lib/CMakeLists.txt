##############################################################################################
# This file is part of OpenAirLink.

# OpenAirLink is free software: you can redistribute it and/or modify it under the terms of 
# the GNU General Public License as published by the Free Software Foundation, either 
# version 3 of the License, or (at your option) any later version.

# OpenAirLink is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along with OpenAirLink.
# If not, see <https://www.gnu.org/licenses/>.
##############################################################################################

########################################################################
# Setup library
########################################################################
#include(GrPlatform) #define LIB_SUFFIX

# List any C++ sources here. If there are no sources (e.g., because there
# is no block controller), then this directory will be skipped.
list(APPEND rfnoc_openairlink_sources
    shiftright_block_control.cpp
)
if(NOT rfnoc_openairlink_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif()

########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/lib
    ${CMAKE_BINARY_DIR}/include
    ${UHD_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)

add_library(rfnoc-openairlink SHARED
    ${rfnoc_openairlink_sources}
)
target_link_libraries(rfnoc-openairlink
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GNURADIO_ALL_LIBRARIES}
    ${ETTUS_LIBRARIES}
)
set_target_properties(rfnoc-openairlink
    PROPERTIES DEFINE_SYMBOL "rfnoc_airlink_EXPORTS")

########################################################################
# Install built library files
########################################################################
install(TARGETS rfnoc-openairlink
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)