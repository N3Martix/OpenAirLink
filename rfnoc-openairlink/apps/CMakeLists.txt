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

# This app needs Boost
set(BOOST_REQUIRED_COMPONENTS
    program_options
    system
)
if(MSVC)
    set(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    if(BOOST_ALL_DYN_LINK)
        add_definitions(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    else(BOOST_ALL_DYN_LINK)
        set(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    endif(BOOST_ALL_DYN_LINK)
endif(MSVC)
find_package(Boost 1.65 REQUIRED ${BOOST_REQUIRED_COMPONENTS})

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(init_shiftright_block
    init_shiftright_block.cpp
)
target_link_libraries(init_shiftright_block
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
    -Wl,--no-as-needed
    rfnoc-openairlink

)

add_executable(oal_single
oal_single.cpp
)
target_link_libraries(oal_single
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
    -Wl,--no-as-needed
    rfnoc-openairlink

)
target_compile_definitions(oal_single PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(oal_dual
oal_dual.cpp
)
target_link_libraries(oal_dual
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
    -Wl,--no-as-needed
    rfnoc-openairlink
)

target_compile_definitions(oal_dual PRIVATE CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")