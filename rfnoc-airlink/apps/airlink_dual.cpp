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

#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/fir_filter_block_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/safe_main.hpp>
#include <rfnoc/airlink/shiftright_block_control.hpp>
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
using rfnoc::airlink::shiftright_block_control;
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
    double rfa_freq, rfb_freq, rx_gain, tx_gain, rx_bw, tx_bw, update_t, print_t, scruni_t;

    // constant variable
    std::string rfa_blkid = "0/Radio#0";
    std::string rfb_blkid = "0/Radio#1";
    std::string fir0_id   = "0/FIR#0";
    std::string shift0_id = "0/Shiftright#0";
    std::string fir1_id   = "0/FIR#1";
    std::string shift1_id = "0/Shiftright#1";

    double setup_time = 0.1;
    uint32_t bit_shift = 0;
    std::vector<int16_t> fir_coeffs;
    fir_coeffs.push_back(32767);

    size_t rfa_chan = 0;  // Channel index
    size_t rfb_chan = 0;
    size_t spp      = 32; // Samples per packet (reduce for lower latency)

    bool rx_timestamps = false; // Set timestamps on RX
    bool use_script    = false;

    // setup config path
    std::string root = CMAKE_SOURCE_DIR;
    std::string config_path_manually = root + "/channel_control/emu_dual.csv";
    std::string config_path_script   = root + "/channel_control/emu_dual_script.csv";
    std::ifstream config_in;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "UHD device address args")
        ("rfa-freq", po::value<double>(&rfa_freq)->default_value(3619.2e6), "Rx RF center frequency in Hz")
        ("rfb-freq", po::value<double>(&rfb_freq)->default_value(3619.2e6), "Tx RF center frequency in Hz")
        ("rx-gain", po::value<double>(&rx_gain)->default_value(0.0), "Rx RF center gain in Hz")
        ("tx-gain", po::value<double>(&tx_gain)->default_value(20.0), "Tx RF center gain in Hz")
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
    uhd::rfnoc::block_id_t rfa_radio_ctrl_id(rfa_blkid);
    uhd::rfnoc::block_id_t rfb_radio_ctrl_id(rfb_blkid);

    uhd::rfnoc::block_id_t fir0_ctrl_id(fir0_id);
    uhd::rfnoc::block_id_t fir1_ctrl_id(fir1_id);
    uhd::rfnoc::block_id_t shift0_ctrl_id(shift0_id);
    uhd::rfnoc::block_id_t shift1_ctrl_id(shift1_id);

    // This next line will fail if the radio is not actually available
    uhd::rfnoc::radio_control::sptr rfa_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(rfa_radio_ctrl_id);
    uhd::rfnoc::radio_control::sptr rfb_radio_ctrl =
        graph->get_block<uhd::rfnoc::radio_control>(rfb_radio_ctrl_id);
    std::cout << "Using RF A radio " << rfa_radio_ctrl_id << ", channel " << rfa_chan
              << std::endl;
    std::cout << "Using RF B radio " << rfb_radio_ctrl_id << ", channel " << rfb_chan
              << std::endl;
    size_t rfa_mb_idx = rfa_radio_ctrl_id.get_device_no();

    // Create FIR filter
    uhd::rfnoc::fir_filter_block_control::sptr fir0_ctrl;
    fir0_ctrl = graph->get_block<uhd::rfnoc::fir_filter_block_control>(fir0_ctrl_id);
    uhd::rfnoc::fir_filter_block_control::sptr fir1_ctrl;
    fir1_ctrl = graph->get_block<uhd::rfnoc::fir_filter_block_control>(fir1_ctrl_id);

    // Create Shiftright block
    rfnoc::airlink::shiftright_block_control::sptr sr0_ctrl;
    sr0_ctrl = graph->get_block<rfnoc::airlink::shiftright_block_control>(shift0_ctrl_id);
    rfnoc::airlink::shiftright_block_control::sptr sr1_ctrl;
    sr1_ctrl = graph->get_block<rfnoc::airlink::shiftright_block_control>(shift1_ctrl_id);

    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Connect block chain from the RF A radio to the RF B radio
    uhd::rfnoc::connect_through_blocks(
        graph, rfa_radio_ctrl_id, rfa_chan, fir0_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, fir0_ctrl_id, 0, shift0_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, shift0_ctrl_id, 0, rfb_radio_ctrl_id, rfb_chan, false);

    uhd::rfnoc::connect_through_blocks(
        graph, rfb_radio_ctrl_id, rfa_chan, fir1_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, fir1_ctrl_id, 0, shift1_ctrl_id, 0, false);
    uhd::rfnoc::connect_through_blocks(
        graph, shift1_ctrl_id, 0, rfa_radio_ctrl_id, rfb_chan, true);
    graph->commit();

    rfa_radio_ctrl->enable_rx_timestamps(rx_timestamps, rfa_chan);
    rfb_radio_ctrl->enable_rx_timestamps(rx_timestamps, rfb_chan);
    rfa_radio_ctrl->set_rx_dc_offset(true, rfa_chan); // Set up DC offset calibration
    rfb_radio_ctrl->set_rx_dc_offset(true, rfb_chan);

    // Set up FIR Filter
    fir0_ctrl->set_coefficients(fir_coeffs, 0);
    fir1_ctrl->set_coefficients(fir_coeffs, 0);
    std::cout << boost::format("Max FIR taps supported: %f") % (fir0_ctrl->get_max_num_coefficients())
              << std::endl;

    // Set up Shiftright
    sr0_ctrl->set_shiftright_value(bit_shift);
    sr1_ctrl->set_shiftright_value(bit_shift);

    // show sample rate
    double rate;
    rate = rfa_radio_ctrl->get_rate();
    std::cout << boost::format("Sample Rate: %f Msps...") % (rate / 1e6) << std::endl;

    // set the center frequency
    rfa_radio_ctrl->set_rx_frequency(rfa_freq, rfa_chan);
    rfa_radio_ctrl->set_tx_frequency(rfa_freq, rfa_chan);
    std::cout << boost::format("Actual RF A central Freq: %f MHz...") % (rfa_radio_ctrl->get_rx_frequency(rfa_chan) / 1e6) << std::endl;

    rfb_radio_ctrl->set_tx_frequency(rfb_freq, rfb_chan);
    rfb_radio_ctrl->set_rx_frequency(rfb_freq, rfb_chan);
    std::cout << boost::format("Actual RF B central Freq: %f MHz...") % (rfb_radio_ctrl->get_tx_frequency(rfb_chan) / 1e6) << std::endl;

    // set the rf gain
    rfa_radio_ctrl->set_rx_gain(rx_gain, rfa_chan);
    rfb_radio_ctrl->set_rx_gain(rx_gain, rfb_chan);
    std::cout << boost::format("Actual RX Gain: %f dB...") % rfa_radio_ctrl->get_rx_gain(rfa_chan)
                << std::endl;

    rfb_radio_ctrl->set_tx_gain(tx_gain, rfb_chan);
    rfa_radio_ctrl->set_tx_gain(tx_gain, rfa_chan);
    std::cout << boost::format("Actual TX Gain: %f dB...") % rfb_radio_ctrl->get_tx_gain(rfb_chan)
                << std::endl;

    // set the IF filter bandwidth
    rfa_radio_ctrl->set_rx_bandwidth(rx_bw, rfa_chan);
    rfb_radio_ctrl->set_rx_bandwidth(rx_bw, rfb_chan);
    std::cout << boost::format("Actual RX Bandwidth: %f MHz...")
                        % (rfa_radio_ctrl->get_rx_bandwidth(rfa_chan) / 1e6)
                << std::endl;

    rfb_radio_ctrl->set_tx_bandwidth(tx_bw, rfb_chan);
    rfa_radio_ctrl->set_tx_bandwidth(tx_bw, rfa_chan);
    std::cout << boost::format("Actual TX Bandwidth: %f MHz...")
                        % (rfb_radio_ctrl->get_tx_bandwidth(rfb_chan) / 1e6)
                << std::endl;

    // set the antennas
    rfa_radio_ctrl->set_property<int>("spp", spp, 0);
    rfb_radio_ctrl->set_property<int>("spp", spp, 0);
    spp = rfa_radio_ctrl->get_property<int>("spp", 0);
    std::cout << "Samples per packet: " << spp << std::endl;

    // Allow for some setup time
    std::this_thread::sleep_for(1s * setup_time);

    // Arm SIGINT handler
    std::signal(SIGINT, &sig_int_handler);

    // Start streaming 
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec =
        graph->get_mb_controller(rfa_mb_idx)->get_timekeeper(rfa_mb_idx)->get_time_now()
        + setup_time;
    std::cout << "Issuing start stream cmd..." << std::endl;
    rfa_radio_ctrl->issue_stream_cmd(stream_cmd, rfa_chan);
    rfb_radio_ctrl->issue_stream_cmd(stream_cmd, rfb_chan);
    std::cout << "Channel emulator is now running..." << std::endl;
    std::cout << std::endl;

    // Keep running and update channel
    std::string fir;
    std::string bit;
    std::vector<int16_t> used_coeffs;

    // Check if script used
    if (vm.count("script")) {
        use_script = true;
        std::cout << "Using Script Config..." << std::endl;
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
                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');
                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir0_ctrl->set_coefficients(fir_coeffs, 0);
                sr0_ctrl->set_shiftright_value(bit_shift);

                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');
                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir1_ctrl->set_coefficients(fir_coeffs, 0);
                sr1_ctrl->set_shiftright_value(bit_shift);

                // Check if FIR & RS coeffs updated
                step += 1;
                std::cout << std::endl;
                std::cout << boost::format("Script Step: %d   ") % (step) 
                          << boost::format("Running Time: %.3fs") % (elapsed_time) << std::endl;

                std::cout << "Channel RF A to RF B:" << std::endl;
                bit_shift = sr0_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift0 bits: %d    ") % (bit_shift);
                used_coeffs = fir0_ctrl->get_coefficients();
                std::cout << boost::format("FIR0 Coeffs:");
                for (int16_t i: used_coeffs) std::cout << i << ' ';
                std::cout << std::endl;

                std::cout << "Channel RF B to RF A:" << std::endl;
                bit_shift = sr1_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift1 bits: %d    ") % (bit_shift);
                used_coeffs = fir1_ctrl->get_coefficients();
                std::cout << boost::format("FIR1 Coeffs:");
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
        std::cout << "Warning: Could not open the script config at '" << config_path_script << "', using manually config instead." << std::endl;

        while (not stop_signal_called) {
            if (is_csv_valid(config_path_manually)) {
                config_in.open(config_path_manually);

                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');
                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir0_ctrl->set_coefficients(fir_coeffs, 0);
                sr0_ctrl->set_shiftright_value(bit_shift);

                std::getline(config_in, fir, ',');
                std::getline(config_in, bit, ',');
                fir_coeffs = fir_parser(fir);
                bit_shift  = static_cast<uint32_t>(std::stoi(bit));

                fir1_ctrl->set_coefficients(fir_coeffs, 0);
                sr1_ctrl->set_shiftright_value(bit_shift);

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
                std::cout << boost::format("Running Time: %fs") % (elapsed_time) << std::endl;

                std::cout << "Channel RF A to RF B:" << std::endl;
                bit_shift = sr0_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift0 bits: %d    ") % (bit_shift);
                used_coeffs = fir0_ctrl->get_coefficients();
                std::cout << boost::format("FIR0 Coeffs:");
                for (int16_t i: used_coeffs) std::cout << i << ' ';
                std::cout << std::endl;

                std::cout << "Channel RF B to RF A:" << std::endl;
                bit_shift = sr1_ctrl->get_shiftright_value();
                std::cout << boost::format("Shift1 bits: %d    ") % (bit_shift);
                used_coeffs = fir1_ctrl->get_coefficients();
                std::cout << boost::format("FIR1 Coeffs:");
                for (int16_t i: used_coeffs) std::cout << i << ' ';
                std::cout << std::endl;
            }
        }
    }

    // Stop radio
    std::cout << std::endl;
    stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
    std::cout << "Issuing stop stream cmd..." << std::endl;
    rfa_radio_ctrl->issue_stream_cmd(stream_cmd, rfa_chan);
    rfb_radio_ctrl->issue_stream_cmd(stream_cmd, rfb_chan);
    std::cout << "Done" << std::endl << std::endl;
    // Allow for the samples and ACKs to propagate
    std::this_thread::sleep_for(100ms);

    return EXIT_SUCCESS;
}