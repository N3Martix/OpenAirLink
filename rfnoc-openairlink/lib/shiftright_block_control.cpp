/**
    This file is part of OpenAirLink.

    OpenAirLink is free software: you can redistribute it and/or modify it under the terms of 
    the GNU General Public License as published by the Free Software Foundation, either 
    version 3 of the License, or (at your option) any later version.

    OpenAirLink is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with OpenAirLink.
    If not, see <https://www.gnu.org/licenses/>.
**/

#include <rfnoc/airlink/shiftright_block_control.hpp>

#include <uhd/rfnoc/defaults.hpp>
#include <uhd/rfnoc/registry.hpp>

using namespace rfnoc::airlink;
using namespace uhd::rfnoc;

const uint32_t shiftright_block_control::REG_SHIFTRIGHT_VALUE = 0x00;

class shiftright_block_control_impl : public shiftright_block_control
{
public:
    RFNOC_BLOCK_CONSTRUCTOR(shiftright_block_control) {}

    void set_shiftright_value(const uint32_t shiftright)
    {
        regs().poke32(REG_SHIFTRIGHT_VALUE, shiftright);
    }

    uint32_t get_shiftright_value()
    {
        return regs().peek32(REG_SHIFTRIGHT_VALUE);
    }

private:
};

UHD_RFNOC_BLOCK_REGISTER_DIRECT(
    shiftright_block_control, 0x02d024, "Shiftright", CLOCK_KEY_GRAPH, "bus_clk")
