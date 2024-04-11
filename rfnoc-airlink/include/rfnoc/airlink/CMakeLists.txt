##############################################################################################
# This file is part of AirLink.

# AirLink is free software: you can redistribute it and/or modify it under the terms of 
# the GNU General Public License as published by the Free Software Foundation, either 
# version 3 of the License, or (at your option) any later version.

# AirLink is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along with AirLink.
# If not, see <https://www.gnu.org/licenses/>.
##############################################################################################

# List all header files here (UHD and GNU Radio)
install(
    FILES
    shiftright_block_control.hpp
    DESTINATION include/rfnoc/shiftright
    COMPONENT headers
)
