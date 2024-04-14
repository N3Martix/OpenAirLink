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

# This macro will tell CMake that this directory contains an RFNoC block. It
# will parse Makefile.srcs to see which files need to be installed, and it will
# register a testbench target for this directory.
RFNOC_REGISTER_BLOCK_DIR()

# This will do the same, but it will skip the testbench target.
#RFNOC_REGISTER_BLOCK_DIR(NOTESTBENCH)


