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

#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/fir_filter_block_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/safe_main.hpp>
#include <rfnoc/openairlink/shiftright_block_control.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

namespace po = boost::program_options;
using uhd::rfnoc::radio_control;
using uhd::rfnoc::fir_filter_block_control;
using rfnoc::openairlink::shiftright_block_control;
using namespace std::chrono_literals;

/****************************************************************************
 * SIGINT handling
 ***************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

/****************************************************************************
 * String to FIR Coeffs
 ***************************************************************************/
std::vector<int16_t> fir_parser(std::string input)
{
    std::istringstream iss(input);
    std::vector<int16_t> fir_coeffs;
    int temp;

    while (iss >> temp) {
        fir_coeffs.push_back(static_cast<int16_t>(temp));
    }

    return fir_coeffs;
}

/****************************************************************************
 * Utility function to trim whitespace from both ends of a string
 ***************************************************************************/
std::string space_trim(const std::string& str) {
    std::string out = str;
    out.erase(out.begin(), std::find_if(out.begin(), out.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    out.erase(std::find_if(out.rbegin(), out.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), out.end());
    return out;
}

/****************************************************************************
 * Verify The condition of CSV config file
 ***************************************************************************/
bool is_csv_valid(const std::string& path) {
    std::ifstream target_csv;
    bool valid;

    target_csv.open(path);
    valid = !target_csv.fail();
    target_csv.close();

    return valid;
}

/****************************************************************************
 * main
 ***************************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, rx_ant, tx_ant;
    double rx_freq, tx_freq, rx_gain, tx_gain, rx_bw, tx_bw, update_t, print_t, scruni_t;

    // constant variable
    std::string rx_blockid = "0/Radio#1";
    std::string tx_blockid = "0/Radio#0";
    std::string fir_id     = "0/FIR#1";
    std::string shift_id   = "0/Shiftright#1";

    double setup_time = 0.1;
    uint32_t bit_shift = 0;
    std::vector<int16_t> fir_coeffs;
    fir_coeffs.push_back(32767);

    size_t rx_chan = 0;  // Channel index
    size_t tx_chan = 0;
    size_t spp     = 32; // Samples per packet (reduce for lower latency)
    
    bool rx_timestamps = false; // Set timestamps on RX
    bool use_script    = false;

    // setup config path
    std::string root = CMAKE_SOURCE_DIR;
    std::string config_path_manually = root + "/channel_control/chan_singel_manually.csv";
    std::string config_path_script = root + "/channel_control/chan_singel_script.csv";
    std::ifstream config_in;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "UHD device address args")
        ("rx-freq", po::value<double>(&rx_freq)->default_value(3619.2e6), "Rx RF center frequency in Hz")
        ("tx-freq", po::value<double>(&tx_freq)->default_value(3619.2e6), "Tx RF center frequency in Hz")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(0.0), "Rx RF center gain in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(0.0), "Tx RF center gain in Hz")
        ("rx-bw", po::value<double>(&rx_bw)->default_value(80e6), "RX analog frontend filter bandwidth in Hz")
        ("tx-bw", po::value<double>(&tx_bw)->default_value(80e6), "TX analog frontend filter bandwidth in Hz")
        ("udt", po::value<double>(&update_t)->default_value(1), "Time period to update emulator channel")
        ("prt", po::value<double>(&print_t)->default_value(5), "Time period to output emulator channel")
        ("script", "Use channel script config")
        ("scr-t", po::value<double>(&scruni_t)->default_value(0.2), "Time period to output emulator channel")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Channel Emulator Singel Link %s") % desc << std::endl;
        std::cout
            << std::endl
            << "This application runs channel emulation using RFNoC.\n"
            << std::endl;
        return ~0;
    }

    /************************************************************************
     * Create device and block controls
     ***********************************************************************/
    std::cout << std::endl;
    std::cout << boost::format("Creating the RFNoC graph with args: %s...") % args
              << std::endl;
    uhd::rfnoc::rfnoc_graph::sptr graph = uhd::rfnoc::rfnoc_graph::make(args);

    // Create handles for radio objects
    uhd::rfnoc::block_id_t rx_radio_ctrl_id(rx_blockid);
    uhd::rfnoc::block_id_t tx_radio_ctrl_id(tx_blockid);
    uhd::rfnoc::block_id_t fir_ctrl_id(fir_id);
    uhd::rfnoc::block_id_t shift_ctrl_id(shift_id);

    // This next line will fail if the radio is not actually available
    uhd::rfnoc::radio_control::sptr rx_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(rx_radio_ctrl_id);
    uhd::rfnoc::radio_control::sptr tx_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(tx_radio_ctrl_id);
    std::cout << "Using RX radio " << rx_radio_ctrl_id << ", channel " << rx_chan
              << std::endl;
    std::cout << "Using TX radio " << tx_radio_ctrl_id << ", channel " << tx_chan
              << std::endl;
    size_t rx_mb_idx = rx_radio_ctrl_id.get_device_no();

    // Create FIR filter
    uhd::rfnoc::fir_filter_block_control::sptr fir_ctrl;
    fir_ctrl = graph->get_block<uhd::rfnoc::fir_filter_block_control>(fir_ctrl_id);

    // Create Shiftright block
    rfnoc::openairlink::shiftright_block_control::sptr sr_ctrl;
    sr_ctrl = graph->get_block<rfnoc::openairlink::shiftright_block_control>(shift_ctrl_id);

    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Only forward properties once per block in the chain. In the case of
    // looping back to a single radio block, skip property propagation after
    // traversing back to the starting point of the chain.
    const bool skip_pp = rx_radio_ctrl_id == tx_radio_ctrl_id;
    // Connect block chain from the RF A radio to the RF B radio

    uhd::rfnoc::connect_through_blocks(
        graph, rx_radio_ctrl_id, rx_chan, fir_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, fir_ctrl_id, 0, shift_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, shift_ctrl_id, 0, tx_radio_ctrl_id, tx_chan, skip_pp);
    graph->commit();

    rx_radio_ctrl->enable_rx_timestamps(rx_timestamps, rx_chan);
    rx_radio_ctrl->set_rx_dc_offset(true, rx_chan);  // Set up DC offset calibration

    // Set up FIR Filter
    fir_ctrl->set_coefficients(fir_coeffs, 0);
    std::cout << boost::format("Max FIR taps supported: %f") % (fir_ctrl->get_max_num_coefficients())
              << std::endl;

    // Set up Shiftright
    sr_ctrl->set_shiftright_value(bit_shift);

    // show sample rate
    double rate;
    std::cout << boost::format("Sample Rate: %f Msps...") % (rate / 1e6) << std::endl;

    // set the center frequency
    std::cout << boost::format("Setting RX Freq: %f MHz...     ") % (rx_freq / 1e6);
    rx_radio_ctrl->set_rx_frequency(rx_freq, rx_chan);
    std::cout << boost::format("Actual RX Freq: %f MHz...")
        % (rx_radio_ctrl->get_rx_frequency(rx_chan) / 1e6) << std::endl;
    
    std::cout << boost::format("Setting TX Freq: %f MHz...     ") % (tx_freq / 1e6);
    tx_radio_ctrl->set_tx_frequency(tx_freq, tx_chan);
    std::cout << boost::format("Actual TX Freq: %f MHz...")
        % (tx_radio_ctrl->get_tx_frequency(tx_chan) / 1e6) << std::endl;

    // set the rf gain
    rx_radio_ctrl->set_rx_gain(rx_gain, rx_chan);
    std::cout << boost::format("Setting RX Gain: %f dB...") % rx_radio_ctrl->get_rx_gain(rx_chan)
              << std::endl;
    tx_radio_ctrl->set_tx_gain(tx_gain, tx_chan);
    std::cout << boost::format("Setting TX Gain: %f dB...") % tx_radio_ctrl->get_tx_gain(tx_chan)
              << std::endl;

    // set the IF filter bandwidth
    rx_radio_ctrl->set_rx_bandwidth(rx_bw, rx_chan);
    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (rx_radio_ctrl->get_rx_bandwidth(rx_chan) / 1e6)
              << std::endl;
    tx_radio_ctrl->set_tx_bandwidth(tx_bw, tx_chan);
    std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % (tx_radio_ctrl->get_tx_bandwidth(tx_chan) / 1e6)
              << std::endl;

    // set the antennas
    rx_radio_ctrl->set_property<int>("spp", spp, 0);
    spp = rx_radio_ctrl->get_property<int>("spp", 0);
    std::cout << "Samples per packet: " << spp << std::endl;

    /************************************************************************
     * Run The Emulator
     ***********************************************************************/

    // Allow for some setup time
    std::this_thread::sleep_for(1s * setup_time);

    // Arm SIGINT handler
    std::signal(SIGINT, &sig_int_handler);

    // Start streaming 
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec =
        graph->get_mb_controller(rx_mb_idx)->get_timekeeper(rx_mb_idx)->get_time_now()
        + setup_time;
    std::cout << "Issuing start stream cmd..." << std::endl;
    rx_radio_ctrl->issue_stream_cmd(stream_cmd, rx_chan);
     
    std::cout << std::endl;
    std::cout << "**********Emulation is Now Running**********" << std::endl;

    // Keep running and update channel
    std::string fir;
    std::string bit;
    std::vector<int16_t> used_coeffs;

    // Check if script used
    if (vm.count("script")) {
        use_script = true;
        std::cout << "Using Script Mode..." << std::endl;
    }
    else {
        std::cout << "Using Manual Mode..." << std::endl;
    }

    double elapsed_time = 0.0;
    if (use_script && is_csv_valid(config_path_script)) {
        config_in.open(config_path_script);

        int step = 0;
        double curr_index;
        std::string index;

        std::getline(config_in, index, ',');
        curr_index  = static_cast<double>(std::stod(index));

        std::cout << boost::format("Script starts at elapsed time: %.3fs") % (curr_index) << std::endl;
        std::cout << "Press Enter to start..." << std::endl;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');        

        while (not stop_signal_called) {
            if (elapsed_time >= curr_index) {
                std::cout << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << std::endl;
                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');

                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir_ctrl->set_coefficients(fir_coeffs, 0);
                sr_ctrl->set_shiftright_value(bit_shift);

                // Check if FIR & RS coeffs updated
                step += 1;
                std::cout << std::endl;
                std::cout << boost::format("Script Step: %d   ") % (step) 
                          << boost::format("Running Time: %.3fs") % (elapsed_time) << std::endl;

                bit_shift = sr_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift bits: %d    ") % (bit_shift);
                used_coeffs = fir_ctrl->get_coefficients();
                std::cout << boost::format("FIR Coeffs:");
                for (int16_t i: used_coeffs) std::cout << i << ' ';
                std::cout << std::endl;

                // Get next config index, keep current config if reach end of script
                std::getline(config_in, index, ',');
                index = space_trim(index); // Trim whitespace
        
                if (index.compare("eos") != 0) {
                    curr_index = static_cast<double>(std::stod(index));
                } else {
                    curr_index = std::numeric_limits<double>::infinity();
                    std::cout << "Reached end of Script, keep the current config..." << std::endl << std::endl;
                }
            }

            // Update Running Time & Sleep for update period
            std::this_thread::sleep_for(1000ms * scruni_t);
            elapsed_time += scruni_t;
            std::cout << '.' << std::flush;
        }
        config_in.close();
    }
    else {
        if (use_script) {
            std::cout << "Warning: Could not open the script config at '" << config_path_script << "', using manually config instead." << std::endl;
        }

        while (not stop_signal_called) {
            if (is_csv_valid(config_path_manually)) {
                config_in.open(config_path_manually);

                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');
                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir_ctrl->set_coefficients(fir_coeffs, 0);
                sr_ctrl->set_shiftright_value(bit_shift);

                config_in.close();
            }
            else {
                std::cout << "Warning: Could not open the config at '" << config_path_manually << "', use default/previous config." << std::endl;
            }
            
            // Update Running Time & Sleep for update period
            std::this_thread::sleep_for(1000ms * update_t);
            elapsed_time += update_t;
            std::cout << '.' << std::flush;

            // Check FIR & RS coeffs with print period
            if (std::fmod(elapsed_time, print_t) == 0) {
                std::cout << std::endl;
                std::cout << boost::format("Running Time: %.3fs") % (elapsed_time) << std::endl;

                bit_shift = sr_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift bits: %d    ") % (bit_shift);
                used_coeffs = fir_ctrl->get_coefficients();
                std::cout << boost::format("FIR Coeffs:");
                for (int16_t i: used_coeffs) std::cout << i << ' ';
                std::cout << std::endl;
            } 
        }
    }

    // Stop radio
    std::cout << std::endl;
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    std::cout << "Issuing stop stream cmd..." << std::endl;
    rx_radio_ctrl->issue_stream_cmd(stream_cmd, rx_chan);
    std::cout << "Done" << std::endl << std::endl;
    // Allow for the samples and ACKs to propagate
    std::this_thread::sleep_for(100ms);

    return EXIT_SUCCESS;
}
