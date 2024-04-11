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

# List Makefile.srcs here (which needs to point to the individual blocks!) as
# well as any non-block specific HDL files that should get installed alongside
# the rest of the FPGA/Verilog/VHDL/HDL files. Only list files that are required
# for synthesis, testbench-specific files do not get installed and thus do not
# have to be listed (it won't hurt, it will just clutter your share/ directory).
# Don't list the files in the block subdirectories, though, they will get added
# below.
install(FILES
    Makefile.srcs
    DESTINATION ${PROJECT_DATA_DIR}/fpga
    COMPONENT fpga
)

# Now call add_subdirectory() for every block subdir
add_subdirectory(rfnoc_block_shiftright)

