/*
 * Copyright 2017 by Lukas Lao Beyer <lukas@electronics.kitchen>
 *
 * This file is part of libfreesrp.
 *
 * libfreesrp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * libfreesrp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with libfreesrp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <adsdr.hpp>
#include "adsdr_impl.h"

namespace ADSDR {

    ADSDR::ADSDR::ADSDR(std::string serial_number)
    {
        _impl.reset(new ADSDR_impl(serial_number));
    }
    
    ADSDR::~ADSDR() = default;
    
    std::vector<std::string> ADSDR::list_connected() { return ADSDR_impl::list_connected(); }

    bool ADSDR::init_sdr() { return _impl->init_sdr(); }
    
    bool ADSDR::fpga_loaded() { return _impl->fpga_loaded(); }
    fpga_status ADSDR::load_fpga(std::string filename) { return _impl->load_fpga(filename); }
    
    void ADSDR::start_rx(std::function<void(const std::vector<sample> &)> rx_callback) { _impl->start_rx(rx_callback); }
    void ADSDR::stop_rx() { _impl->stop_rx(); }
    
    void ADSDR::start_tx(std::function<void(std::vector<sample> &)> tx_callback) { _impl->start_tx(tx_callback); }
    void ADSDR::stop_tx() { _impl->stop_tx(); }
    
    unsigned long ADSDR::available_rx_samples() {return _impl->available_rx_samples(); }
    bool ADSDR::get_rx_sample(sample &s) { return _impl->get_rx_sample(s); }
    
    bool ADSDR::submit_tx_sample(sample &s) { return _impl->submit_tx_sample(s); }
    
    command ADSDR::make_command(command_id id, double param) const { return _impl->make_command(id, param); }
    response ADSDR::send_cmd(command c) const { return _impl->send_cmd(c); }
    
    adsdr_version ADSDR::version() { return _impl->version(); }
    
}
