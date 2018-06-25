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

#ifndef __LIBADSDR_ADSDR_IMPL_HPP__
#define __LIBADSDR_ADSDR_IMPL_HPP__


#include <adsdr.hpp>
#include "readerwriterqueue/readerwriterqueue.h"

#include "devioifce.h"
#include "ad9361_tuner.h"

#include "libusb.h"


enum fx3cmd {
    REG_SPI_READ8       = 0xB5,
    REG_SPI_WRITE8      = 0xB6,
    DEVICE_START        = 0xBA,
    DEVICE_STOP         = 0xBB,
    DEVICE_RESET        = 0xB3
};

namespace ADSDR
{
    class ADSDR_impl : public DeviceControlIOIfce
    {
    public:
        ADSDR_impl(std::string serial_number = "");
        ~ADSDR_impl();

        static std::vector<std::string> list_connected();
	
        bool fpga_loaded();
        fpga_status load_fpga(std::string filename);

        bool init_sdr();
	
#if 0
        std::shared_ptr<rx_tx_buf> rx();
        void tx(std::shared_ptr<rx_tx_buf> buf);
#endif

        void start_rx(std::function<void(const std::vector<sample> &)> rx_callback = {});
        void stop_rx();

        void start_tx(std::function<void(std::vector<sample> &)> tx_callback = {});
        void stop_tx();

        unsigned long available_rx_samples();
        bool get_rx_sample(sample &s);

        bool submit_tx_sample(sample &s);

        command make_command(command_id id, double param) const;
        response send_cmd(command cmd);

        adsdr_version version();

    protected:
        // DeviceControlIOIfce interface
        virtual uint8_t peek8( uint32_t register_address24);
        virtual void poke8( uint32_t register_address24, uint8_t value );


        int send24bitSPI(uint16_t addr, uint8_t data);
        int read24bitSPI(uint16_t addr, uint8_t* data);

        int txControlToDevice(uint8_t* src, uint32_t size8, uint8_t cmd, uint16_t wValue, uint16_t wIndex);
        int txControlFromDevice(uint8_t* dest, uint32_t size8 , uint8_t cmd, uint16_t wValue, uint16_t wIndex);

        int ctrlToDevice(uint8_t cmd, uint16_t value, uint16_t index, void* data, size_t data_len);
        int ctrlFromDevice(uint8_t cmd, uint16_t value, uint16_t index, void* dest, size_t data_len);


        int deviceStart();
        int deviceStop();
        int deviceReset();

        //------------------ Commands ------------------------------
        typedef void (ADSDR_impl::*cmd_function)(uint64_t* param, char param_no, char* error, uint64_t* response);

        response ad9364_cmd(int cmd, uint64_t param);
        // Gets current TX LO frequency.
        void get_tx_lo_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the TX LO frequency.
        void set_tx_lo_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current TX sampling frequency.
        void get_tx_samp_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the TX sampling frequency.
        void set_tx_samp_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current TX RF bandwidth.
        void get_tx_rf_bandwidth(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the TX RF bandwidth.
        void set_tx_rf_bandwidth(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current TX attenuation.
        void get_tx_attenuation(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the TX attenuation.
        void set_tx_attenuation(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current TX FIR state.
        void get_tx_fir_en(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the TX FIR state.
        void set_tx_fir_en(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX LO frequency.
        void get_rx_lo_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX LO frequency.
        void set_rx_lo_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX sampling frequency.
        void get_rx_samp_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX sampling frequency.
        void set_rx_samp_freq(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX RF bandwidth.
        void get_rx_rf_bandwidth(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX RF bandwidth.
        void set_rx_rf_bandwidth(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX1 GC mode.
        void get_rx_gc_mode(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX GC mode.
        void set_rx_gc_mode(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX RF gain.
        void get_rx_rf_gain(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX RF gain.
        void set_rx_rf_gain(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Gets current RX FIR state.
        void get_rx_fir_en(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Sets the RX FIR state.
        void set_rx_fir_en(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Enables/disables the datapath. (Puts AD9364 into FDD state/alert state and notifies the rest of the FPGA system) */
        void set_datapath_en(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Get FPGA design version.
        void get_version(uint64_t* param, char param_no, char* error, uint64_t* response);
        // Enables/disables the AD9364's loopback BIST mode
        void set_loopback_en(uint64_t* param, char param_no, char* error, uint64_t* response);

    private:
        void run_rx_tx();

        libusb_transfer *create_rx_transfer(libusb_transfer_cb_fn callback);
        libusb_transfer *create_tx_transfer(libusb_transfer_cb_fn callback);

        static void rx_callback(libusb_transfer *transfer);
        static void tx_callback(libusb_transfer *transfer);

        static int fill_tx_transfer(libusb_transfer *transfer);

        static void decode_rx_transfer(unsigned char *buffer, int actual_length, std::vector<sample> &destination);

        libusb_context* _ctx = nullptr;
        libusb_device_handle *_adsdr_handle = nullptr;

        std::string _fx3_fw_version;

        std::atomic<bool> _run_rx_tx{false};
        std::unique_ptr<std::thread> _rx_tx_worker;

        std::array<libusb_transfer *, ADSDR_RX_TX_TRANSFER_QUEUE_SIZE> _rx_transfers;
        std::array<libusb_transfer *, ADSDR_RX_TX_TRANSFER_QUEUE_SIZE> _tx_transfers;

        static std::function<void(const std::vector<sample> &)> _rx_custom_callback;
        static std::function<void(std::vector<sample> &)> _tx_custom_callback;

        static std::vector<sample> _rx_decoder_buf;
        static std::vector<sample> _tx_encoder_buf;

        static moodycamel::ReaderWriterQueue<sample> _rx_buf;
        static moodycamel::ReaderWriterQueue<sample> _tx_buf;

        std::shared_ptr<ad9361_device_t> m_adsdr;
        ad9361_params m_adsdr_params;
        double m_rx_freq;
        double m_clock_rate;

        std::vector<cmd_function> m_cmd_list;
    };
}

#endif // __LIBADSDR_ADSDR_IMPL_HPP__

