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

// It will see if a USRP is runnging the shiftright block, if so, it will test to see
// if it can change the shiftright bits.

#include <uhd/exception.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/utils/safe_main.hpp>
#include <rfnoc/airlink/shiftright_block_control.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "USRP device address args")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << "Init RFNoC shiftright block " << desc << std::endl;
        std::cout << std::endl
                  << "This application attempts to find a shiftright block in a USRP "
                     "and tries to peek/poke registers..\n"
                  << std::endl;
        return EXIT_SUCCESS;
    }

    // Create RFNoC graph object:
    auto graph = uhd::rfnoc::rfnoc_graph::make(args);

    // Verify we have a shiftright block:
    auto shiftright_blocks = graph->find_blocks<rfnoc::airlink::shiftright_block_control>("");
    if (shiftright_blocks.empty()) {
        std::cout << "No shiftright block found." << std::endl;
        return EXIT_FAILURE;
    }

    auto shiftright_block =
        graph->get_block<rfnoc::airlink::shiftright_block_control>(shiftright_blocks.front());
    if (!shiftright_block) {
        std::cout << "ERROR: Failed to extract block controller!" << std::endl;
        return EXIT_FAILURE;
    }
    constexpr uint32_t new_shiftright_value = 42;
    shiftright_block->set_shiftright_value(new_shiftright_value);
    const uint32_t shiftright_value_read = shiftright_block->get_shiftright_value();

    if (shiftright_value_read != new_shiftright_value) {
        std::cout << "ERROR: Readback of shiftright value not working! "
                  << "Expected: " << new_shiftright_value << " Read: " << shiftright_value_read
                  << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Shiftright value read/write loopback successful!" << std::endl;
    }

    return EXIT_SUCCESS;
}
