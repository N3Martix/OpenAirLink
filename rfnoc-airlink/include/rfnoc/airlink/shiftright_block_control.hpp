/**
    This file is part of AirLink.

    AirLink is free software: you can redistribute it and/or modify it under the terms of 
    the GNU General Public License as published by the Free Software Foundation, either 
    version 3 of the License, or (at your option) any later version.

    AirLink is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with AirLink.
    If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef INCLUDED_RFNOC_AIRLINK_SHIFTRIGHT_BLOCK_CONTROL_HPP
#define INCLUDED_RFNOC_AIRLINK_SHIFTRIGHT_BLOCK_CONTROL_HPP

#include <uhd/config.hpp>
#include <uhd/rfnoc/noc_block_base.hpp>
#include <uhd/types/stream_cmd.hpp>

namespace rfnoc { namespace airlink {

/*! Block controller for the shiftright block: right shifts signal by given bits
 *
 * This block right shifts the signal input with fixed bits.
 */
class UHD_API shiftright_block_control : public uhd::rfnoc::noc_block_base
{
public:
    RFNOC_DECLARE_BLOCK(shiftright_block_control)

    //! The register address of the shiftright bits
    static const uint32_t REG_SHIFTRIGHT_VALUE;

    /*! Set the shiftright bits
     */
    virtual void set_shiftright_value(const uint32_t shiftright) = 0;

    /*! Get the current shiftright bits (read it from the device)
     */
    virtual uint32_t get_shiftright_value() = 0;
};

}} // namespace rfnoc::airlink

#endif /* INCLUDED_RFNOC_AIRLINK_SHIFTRIGHT_BLOCK_CONTROL_HPP */