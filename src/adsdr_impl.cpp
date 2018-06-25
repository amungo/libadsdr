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

#include "adsdr_impl.h"
#include <adsdr.hpp>

#include <cstring>
#include <fstream>
#include <iostream>

#include "fx3deverr.h"


#define ADSDR_SERIAL_DSCR_INDEX 3
#define MAX_SERIAL_LENGTH 256

#define DEV_UPLOAD_TIMEOUT_MS 1000 // Timeout for uploading one block of firmware

using namespace ADSDR;

moodycamel::ReaderWriterQueue<sample> ADSDR_impl::_rx_buf(ADSDR_RX_TX_QUEUE_SIZE);
moodycamel::ReaderWriterQueue<sample> ADSDR_impl::_tx_buf(ADSDR_RX_TX_QUEUE_SIZE);
std::vector<sample> ADSDR_impl::_rx_decoder_buf(ADSDR_RX_TX_BUF_SIZE / ADSDR_BYTES_PER_SAMPLE);
std::function<void(const std::vector<sample> &)> ADSDR_impl::_rx_custom_callback;
std::vector<sample> ADSDR_impl::_tx_encoder_buf(ADSDR_RX_TX_BUF_SIZE / ADSDR_BYTES_PER_SAMPLE);
std::function<void(std::vector<sample> &)> ADSDR_impl::_tx_custom_callback;

ADSDR_impl::ADSDR_impl(std::string serial_number) : m_rx_freq(ad9361_device_t::DEFAULT_RX_FREQ),
                                                    m_clock_rate(DEFAULT_CLOCK_RATE)
{
    //----------------------------------------------------
    m_cmd_list.push_back(&ADSDR_impl::get_tx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_attenuation);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_attenuation);
    m_cmd_list.push_back(&ADSDR_impl::get_tx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_tx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_lo_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_samp_freq);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_rf_bandwidth);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_gc_mode);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_gc_mode);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_rf_gain);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_rf_gain);
    m_cmd_list.push_back(&ADSDR_impl::get_rx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_rx_fir_en);
    m_cmd_list.push_back(&ADSDR_impl::set_datapath_en);
    m_cmd_list.push_back(&ADSDR_impl::get_version);
    m_cmd_list.push_back(&ADSDR_impl::set_loopback_en);
    // ---------------------------------------------------
    libusb_device **devs;

    int ret = libusb_init(&_ctx);

    if(ret < 0)
    {
        throw ConnectionError("libusb init error: error " + std::to_string(ret));
    }

    // Set verbosity level
    libusb_set_debug(_ctx, 3);

    // Retrieve device list
    int num_devs = (int) libusb_get_device_list(_ctx, &devs);
    if(num_devs < 0)
    {
        throw ConnectionError("libusb device list retrieval error");
    }

    // Find ADSDR device
    bool no_match = false;
    
    for(int i = 0; i < num_devs; i++)
    {
        libusb_device_descriptor desc;
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0)
        {
            throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID)
        {
            int ret = libusb_open(devs[i], &_adsdr_handle);
            if(ret != 0)
            {
                throw ConnectionError("libusb could not open found ADSDR USB device: error " + std::to_string(ret));
            }


            // Check if correct serial number
            if(desc.iSerialNumber)
            {
                char serial_num_buf[MAX_SERIAL_LENGTH];
                ret = libusb_get_string_descriptor_ascii(_adsdr_handle, /*ADSDR_SERIAL_DSCR_INDEX*/desc.iSerialNumber, (unsigned char*)serial_num_buf, MAX_SERIAL_LENGTH);
                if(ret < 0)
                {
                    libusb_close(_adsdr_handle);
                    _adsdr_handle = nullptr;
                    throw ConnectionError("1)libusb could not read ADSDR serial number: error " + std::to_string(ret) + "___" + std::to_string(desc.iSerialNumber));
                }
                else
                {
                    std::string dev_serial = std::string(serial_num_buf);
                    if(dev_serial.find(serial_number) != std::string::npos)
                    {
                        // Found!
                        break;
                    }
                    else
                    {
                        no_match = true;
                        libusb_close(_adsdr_handle);
                        _adsdr_handle = nullptr;
                    }
                }
            }
            else
            {
                std::cout << "ADSDR serial number not found !" << std::endl;
            }
        }
    }

    if(no_match && _adsdr_handle == nullptr)
    {
        throw ConnectionError("ADSDR device(s) were found, but did not match specified serial number");
    }
    
    if(_adsdr_handle == nullptr)
    {
        throw ConnectionError("no ADSDR device found");
    }

    // Free the list, unref the devices in it
    libusb_free_device_list(devs, 1);

    // Found a ADSDR device and opened it. Now claim its interface (ID 0).
    ret = libusb_claim_interface(_adsdr_handle, 0);
    if(ret < 0)
    {
        throw ConnectionError("could not claim ADSDR interface");
    }

    // Request ADSDR version number
#if 0
    std::array<unsigned char, ADSDR_USB_CTRL_SIZE> data{};
    ret = libusb_control_transfer(_adsdr_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, ADSDR_GET_VERSION_REQ, 0, 0, data.data(), (uint16_t) data.size(), ADSDR_USB_TIMEOUT);
    if(ret < 0)
    {
        throw ConnectionError("1) ADSDR not responding: error " + std::to_string(ret));
    }
    int transferred = ret;
    _fx3_fw_version = std::string(std::begin(data), std::begin(data) + transferred);
#endif

    for(size_t i = 0; i < _rx_transfers.size(); i++)
    {
        _rx_transfers[i] = create_rx_transfer(&ADSDR_impl::rx_callback);
    }

#if 0
    for(int size_t = 0; i < _tx_transfers.size(); i++)
    {
        _tx_transfers[i] = create_tx_transfer(&ADSDR_impl::tx_callback);
    }
#endif

    // Start libusb event handling
    _run_rx_tx.store(true);

    _rx_tx_worker.reset(new std::thread([this]() {
        run_rx_tx();
    }));
}

ADSDR_impl::~ADSDR_impl()
{
    // TODO: Properly stop all active transfers
    stop_rx();
    stop_tx();

    if(_adsdr_handle != nullptr)
    {
        libusb_release_interface(_adsdr_handle, 0);

        _run_rx_tx.store(false);

        // This will cause libusb_handle_events() in run_rx_tx() to return once
        libusb_close(_adsdr_handle);

        // libusb_handle_events should have returned and the thread can now be joined
        if(_rx_tx_worker != nullptr)
        {
            _rx_tx_worker->join();
        }
    }

    for(libusb_transfer *transfer : _rx_transfers)
    {
        libusb_free_transfer(transfer);
    }

#if 0
    for(libusb_transfer *transfer : _tx_transfers)
    {
        libusb_free_transfer(transfer);
    }
#endif

    if(_ctx != nullptr)
    {
        libusb_exit(_ctx); // close the session
    }
}

bool ADSDR_impl::init_sdr()
{
    m_adsdr = std::make_shared<ad9361_device_t>(this, &m_adsdr_params);
    m_adsdr->initialize();

    return true;
}

std::vector<std::string> ADSDR_impl::list_connected()
{
    libusb_device **devs;
    libusb_context *list_ctx;

    std::vector<std::string> list;

    int ret = libusb_init(&list_ctx);
    if(ret < 0)
    {
        throw ConnectionError("libusb init error: error " + std::to_string(ret));
    }

    // Set verbosity level
    libusb_set_debug(list_ctx, 3);

    // Retrieve device list
    int num_devs = (int) libusb_get_device_list(list_ctx, &devs);
    if(num_devs < 0)
    {
        throw ConnectionError("libusb device list retrieval error");
    }

    // Find all ADSDR devices
    for(int i = 0; i < num_devs; i++)
    {
        libusb_device_descriptor desc;
        int ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0)
        {
            throw ConnectionError("libusb error getting device descriptor: error " + std::to_string(ret));
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID)
        {
	    libusb_device_handle *temp_handle;
            int ret = libusb_open(devs[i], &temp_handle);
            if(ret != 0)
            {
                throw ConnectionError("libusb could not open found ADSDR USB device: error " + std::to_string(ret));
            }

        // Check if correct serial number
        if(desc.iSerialNumber)
        {
            char serial_num_buf[MAX_SERIAL_LENGTH];
            ret = libusb_get_string_descriptor_ascii(temp_handle, /*ADSDR_SERIAL_DSCR_INDEX*/desc.iSerialNumber, (unsigned char *) serial_num_buf, MAX_SERIAL_LENGTH);
            if(ret < 0)
            {
                throw ConnectionError("2) libusb could not read ADSDR serial number: error " + std::to_string(ret));
            }
            else
            {
                list.push_back(std::string(serial_num_buf));
            }
        }
        else
            list.push_back(std::string("12345"));
	    
	    libusb_close(temp_handle);
        }
    }

    libusb_exit(list_ctx);

    return list;
}

bool ADSDR_impl::fpga_loaded()
{
    std::array<unsigned char, ADSDR_USB_CTRL_SIZE> stat_buf{};
    int ret = libusb_control_transfer(_adsdr_handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, ADSDR_FPGA_CONFIG_STATUS, 0, 1, stat_buf.data(), (uint16_t) stat_buf.size(), ADSDR_USB_TIMEOUT);
    if(ret < 0)
    {
        throw ConnectionError("2) ADSDR not responding: error " + std::to_string(ret));
    }
    //int transferred = ret;
    bool fpga_load_success = (bool) stat_buf[0];
    return fpga_load_success;
}

fpga_status ADSDR_impl::load_fpga(std::string filename)
{
    filename = filename;

    return FPGA_CONFIG_SKIPPED; // @camry
}

libusb_transfer* ADSDR_impl::create_rx_transfer(libusb_transfer_cb_fn callback)
{
    libusb_transfer *transfer = libusb_alloc_transfer(0);
    unsigned char *buf = new unsigned char[ADSDR_RX_TX_BUF_SIZE];
    libusb_fill_bulk_transfer(transfer, _adsdr_handle, ADSDR_RX_IN, buf, ADSDR_RX_TX_BUF_SIZE, callback, nullptr, ADSDR_USB_TIMEOUT);

    return transfer;
}

libusb_transfer* ADSDR_impl::create_tx_transfer(libusb_transfer_cb_fn callback)
{
    callback = callback; // warning
#if 0
    libusb_transfer *transfer = libusb_alloc_transfer(0);
    unsigned char *buf = new unsigned char[ADSDR_TX_BUF_SIZE];
    libusb_fill_bulk_transfer(transfer, _adsdr_handle, ADSDR_TX_OUT, buf, ADSDR_TX_BUF_SIZE, callback, nullptr, ADSDR_USB_TIMEOUT);
    //TODO: transfer size
    return transfer;
#endif

    return 0;
}

void ADSDR_impl::rx_callback(libusb_transfer *transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {         
        // Transfer succeeded

        // Decode samples from transfer buffer into _rx_decoder_buf
        decode_rx_transfer(transfer->buffer, transfer->actual_length, _rx_decoder_buf);

        if(_rx_custom_callback)
        {
            // Run the callback function
            _rx_custom_callback(_rx_decoder_buf);
        }
        else
        {
            // No callback function specified, add samples to queue
            for(sample s : _rx_decoder_buf)
            {
                bool success = _rx_buf.try_enqueue(s);
                if(!success)
                {
                    // TODO: overflow! handle this
                }
            }
        }
    }
    else
    {
        // TODO: Handle error

    }

    // Resubmit the transfer
    if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            // TODO: Handle error
        }
    }
}

void ADSDR_impl::tx_callback(libusb_transfer* transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        // Success
        if(transfer->actual_length != transfer->length)
        {
            std::cout << "actual length != length: " << transfer->actual_length << "; " << transfer->length << std::endl;
        }
    }
    else
    {
        // TODO: Handle error
        if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
        {
            std::cerr << "transfer error with status " << transfer->status << std::endl;
        }
    }

    // Resubmit the transfer with new data
    if(transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
        fill_tx_transfer(transfer);
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            // TODO: Handle error
            std::cerr << "transfer submission error with status " << transfer->status << std::endl;
        }
    }
}

void ADSDR_impl::start_rx(std::function<void(const std::vector<sample> &)> rx_callback)
{
    _rx_custom_callback = rx_callback;

    deviceStop();

    //deviceReset();

    for(libusb_transfer *transfer: _rx_transfers)
    {
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            throw ConnectionError("Could not submit RX transfer. libusb error: " + std::to_string(ret));
        }
    }

    deviceStart();
}

void ADSDR_impl::stop_rx()
{
    for(libusb_transfer *transfer: _rx_transfers)
    {
        int ret = libusb_cancel_transfer(transfer);
        if(ret == LIBUSB_ERROR_NOT_FOUND || ret == 0)
        {
            // Transfer cancelled
        }
        else
        {
            // Error
            throw ConnectionError("Could not cancel RX transfer. libusb error: " + std::to_string(ret));
        }
    }
}

void ADSDR_impl::start_tx(std::function<void(std::vector<sample> &)> tx_callback)
{
    _tx_custom_callback = tx_callback;

    // Fill the tx buffer with empty samples
    sample empty_sample{0, 0};
    while(_tx_buf.try_enqueue(empty_sample)) {}

    for(libusb_transfer *transfer: _tx_transfers)
    {
        fill_tx_transfer(transfer);
        int ret = libusb_submit_transfer(transfer);

        if(ret < 0)
        {
            throw ConnectionError("Could not submit TX transfer. libusb error: " + std::to_string(ret));
        }
    }
}

void ADSDR_impl::stop_tx()
{
#if 0
    for(libusb_transfer *transfer: _tx_transfers)
    {
        int ret = libusb_cancel_transfer(transfer);
        if(ret == LIBUSB_ERROR_NOT_FOUND || ret == 0)
        {
            // Transfer cancelled
        }
        else
        {
            // Error
            throw ConnectionError("Could not cancel TX transfer. libusb error: " + std::to_string(ret));
        }
    }
#endif
}

int ADSDR_impl::fill_tx_transfer(libusb_transfer* transfer)
{
    // Fill the transfer buffer with available samples
    transfer->length = ADSDR_TX_BUF_SIZE;

    _tx_encoder_buf.resize(transfer->length/ADSDR_BYTES_PER_SAMPLE);

    if(_tx_custom_callback)
    {
        _tx_custom_callback(_tx_encoder_buf);
    }
    else
    {
        for(sample &s : _tx_encoder_buf)
        {
            int success = _tx_buf.try_dequeue(s);
            if(!success)
            {
                // TODO: Notify of this? Do something else?
                // No data available, fill with zeros
                s.i = 0;
                s.q = 0;
            }
        }
    }

    for(int i = 0; i < transfer->length; i+=ADSDR_BYTES_PER_SAMPLE)
    {
        // SKIP THIS: Convert -1.0 to 1.0 float sample value to signed 16-bit int with range -2048 to 2048
        int16_t signed_i = _tx_encoder_buf[i/ADSDR_BYTES_PER_SAMPLE].i;
        int16_t signed_q = _tx_encoder_buf[i/ADSDR_BYTES_PER_SAMPLE].q;

        // Unsigned 16-bit ints holding the two's-complement 12-bit sample values
        uint16_t raw_i;
        uint16_t raw_q;

        if(signed_i >= 0)
        {
            raw_i = (uint16_t) signed_i;
        }
        else
        {
            raw_i = (((uint16_t) (-signed_i)) ^ ((uint16_t) 0xFFF)) + (uint16_t) 1;
        }

        if(signed_q >= 0)
        {
            raw_q = (uint16_t) signed_q;
        }
        else
        {
            raw_q = (((uint16_t) (-signed_q)) ^ ((uint16_t) 0xFFF)) + (uint16_t) 1;
        }

        // Copy raw i/q data into the buffer
        memcpy(transfer->buffer + i, &raw_q, sizeof(raw_q));
        memcpy(transfer->buffer + i + sizeof(raw_q), &raw_i, sizeof(raw_i));
    }

    return transfer->length;
}

void ADSDR_impl::decode_rx_transfer(unsigned char *buffer, int actual_length, std::vector<sample> &destination)
{
    int16_t* pSamplesIn = (short*)buffer;
    int lenght = actual_length / (sizeof(short) * 2 * 2);
    destination.resize(lenght);
    sample* pSamplesOut = destination.data();

    for(int i = 0; i < lenght; i++)
    {
        pSamplesOut[i].i = pSamplesIn[4*i+0]>>4;
        pSamplesOut[i].q = pSamplesIn[4*i+1]>>4;
    }
}

void ADSDR_impl::run_rx_tx()
{
    while(_run_rx_tx.load())
    {
        libusb_handle_events(_ctx);
    }
}

unsigned long ADSDR_impl::available_rx_samples()
{
    return _rx_buf.size_approx();
}

bool ADSDR_impl::get_rx_sample(sample &s)
{
    return _rx_buf.try_dequeue(s);
}

bool ADSDR_impl::submit_tx_sample(sample &s)
{
    return _tx_buf.try_enqueue(s);
}

command ADSDR_impl::make_command(command_id id, double param) const
{
    command cmd;

    cmd.cmd = id;
    switch(cmd.cmd)
    {
    case SET_TX_LO_FREQ:
    {
        uint64_t cast_param = static_cast<uint64_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_SAMP_FREQ:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_RF_BANDWIDTH:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_ATTENUATION:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_TX_FIR_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_LO_FREQ:
    {
        uint64_t cast_param = static_cast<uint64_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_SAMP_FREQ:
    {
        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_RF_BANDWIDTH:
    {
        std::

        uint32_t cast_param = static_cast<uint32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_GC_MODE:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_RF_GAIN:
    {
        int32_t cast_param = static_cast<int32_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_RX_FIR_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_DATAPATH_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    case SET_LOOPBACK_EN:
    {
        uint8_t cast_param = static_cast<uint8_t>(param);
        memcpy(&cmd.param, &cast_param, sizeof(cast_param));
    }
        break;
    default:
        throw std::runtime_error("make_command error: " + std::to_string(id));
    }

    return cmd;
}

response ADSDR_impl::send_cmd(command cmd)
{    
    response reply;

    if(cmd.cmd < COMMAND_SIZE)
    {
        reply = ad9364_cmd(cmd.cmd, cmd.param);
    }
    std::cout << " Send cmd: " << cmd.cmd << " param: " << cmd.param << std::endl;

    return reply;
}

adsdr_version ADSDR_impl::version()
{
    response res = send_cmd({GET_FPGA_VERSION});
    uint8_t fpga_major_version = ((uint8_t*) &res.param)[0];
    uint8_t fpga_minor_version = ((uint8_t*) &res.param)[1];
    uint8_t fpga_patch_version = ((uint8_t*) &res.param)[2];

    adsdr_version v;
    v.fx3 = _fx3_fw_version;
    v.fpga = std::to_string(fpga_major_version) + "." + std::to_string(fpga_minor_version) + "." + std::to_string(fpga_patch_version);

    return v;
}


//========================================================================================================

int ADSDR_impl::deviceStart()
{
    uint8_t buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    buf[2] = 0xFF;
    return ctrlToDevice(DEVICE_START, 0, 1, buf, 16);
}

int ADSDR_impl::deviceStop()
{
    uint8_t buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    buf[2] = 0xFF;
    return ctrlToDevice(DEVICE_STOP, 0, 1, buf, 16);
}

int ADSDR_impl::deviceReset()
{
    uint8_t buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    buf[2] = 0xFF;
    return ctrlToDevice(DEVICE_RESET, 0, 1, buf, 16);
}


uint8_t ADSDR_impl::peek8( uint32_t register_address24)
{
    uint8_t val = 0;
    if(read24bitSPI(register_address24, &val) == FX3_ERR_OK) {
        return val;
    } else {
        return 0xFF;
    }
}

void ADSDR_impl::poke8( uint32_t register_address24, uint8_t value )
{
    send24bitSPI(register_address24, value);
}


int ADSDR_impl::send24bitSPI(uint16_t addr, uint8_t data)
{
    uint8_t buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    addr |= 0x8000;
    buf[0] = data;
    buf[1] = addr & 0x0ff;
    buf[2] = addr >> 8;

    return ctrlToDevice(REG_SPI_WRITE8, 0, 1, buf, 16);
}

int ADSDR_impl::read24bitSPI(uint16_t addr, uint8_t* data)
{
    uint8_t buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    buf[0] = *data;
    buf[1] = addr & 0x0ff;
    buf[2] = addr >> 8;

    size_t len = 16;
    int success = ctrlFromDevice(REG_SPI_READ8, *data, addr, buf, len);

    *data = buf[0];
    return success;
}

int ADSDR_impl::txControlToDevice(uint8_t* src, uint32_t size8, uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
    if(_adsdr_handle != 0)
    {
        /* From libusb-1.0 documentation:
         * Bits 0:4 determine recipient, see libusb_request_recipient.
         * Bits 5:6 determine type, see libusb_request_type.
         * Bit 7 determines data transfer direction, see libusb_endpoint_direction. */
        uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT;

        /* From libusb-1.0 documentation:
         * If the type bits of bmRequestType are equal to LIBUSB_REQUEST_TYPE_STANDARD
         * then this field refers to libusb_standard_request.
         * For other cases, use of this field is application-specific. */
        uint8_t bRequest = cmd;

        /* From libusb-1.0 documentation:
         * timeout (in millseconds) that this function should wait before giving up
         * due to no response being received.
         * For an unlimited timeout, use value 0. */
        uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

        int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, src, size8, timeout_ms );
        if ( res != ( int ) size8 ) {
            fprintf( stderr, "FX3Dev::txControlToDevice() error %d %s\n", res, libusb_error_name(res) );
            return FX3_ERR_CTRL_TX_FAIL;
        }
        return FX3_ERR_OK;
    }

    return FX3_ERR_NO_DEVICE_FOUND;
}

int ADSDR_impl::txControlFromDevice(uint8_t* dest, uint32_t size8 , uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
    if(_adsdr_handle != 0)
    {
        /* From libusb-1.0 documentation:
         * Bits 0:4 determine recipient, see libusb_request_recipient.
         * Bits 5:6 determine type, see libusb_request_type.
         * Bit 7 determines data transfer direction, see libusb_endpoint_direction. */
        uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN;
        uint8_t bRequest = cmd;
        uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

        int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, dest, size8, timeout_ms );
        if ( res != ( int ) size8 ) {
            fprintf( stderr, "FX3Dev::transferDataFromDevice() error %d %s\n", res, libusb_error_name(res) );
            return FX3_ERR_CTRL_TX_FAIL;
        }
        return FX3_ERR_OK;
    }

    return FX3_ERR_NO_DEVICE_FOUND;
}


int ADSDR_impl::ctrlToDevice(uint8_t cmd, uint16_t value, uint16_t index, void* data, size_t data_len)
{
    uint8_t  dummybuf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint32_t len = 16;
    uint8_t* buf = dummybuf;

    if ( data && data_len != 0 ) {
        buf = (uint8_t*)data;
        len = data_len;
    }

    return txControlToDevice( buf, len, cmd, value, index );
}

int ADSDR_impl::ctrlFromDevice(uint8_t cmd, uint16_t value, uint16_t index, void* dest, size_t data_len)
{
    uint8_t  dummybuf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint32_t len = 16;
    uint8_t* buf = dummybuf;

    if ( dest && data_len != 0 ) {
        buf = (uint8_t*)dest;
        len = data_len;
    }
    return txControlFromDevice( buf, len, cmd, value, index );
}

//=====================================  Commands  ======================================================

response ADSDR_impl::ad9364_cmd(int cmd, uint64_t param)
{
    char param_no = 1;
    char error = CMD_OK;
    uint64_t ret_value = 0;

    (this->*(m_cmd_list[cmd]))(&param, param_no, &error, &ret_value);

    response reply;
    reply.cmd = (command_id)cmd;
    reply.error = (command_err)error;
    reply.param = ret_value;

    return reply;
}

// ************************************************************************
// * @brief Gets current TX LO frequency [Hz].
// *
// * @return None.
// ************************************************************************
void ADSDR_impl::get_tx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_lo_freq?" command
{
    uint64_t lo_freq_hz = 0;

    *error = CMD_OK;
    memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));
    printf("tx_lo_freq=%lu Hz\n\r", lo_freq_hz);
}

// **************************************************************************
// * @brief Sets the TX LO frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_tx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_lo_freq=" command
{
    uint64_t lo_freq_hz = 0;
    //uint32_t port = TXA;

    if(param_no >= 1)
    {
        memcpy(&lo_freq_hz, param, sizeof(lo_freq_hz));
        printf("tx_lo_freq=%lu Hz\n\r", (unsigned long)lo_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_lo_freq: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current sampling frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_tx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_samp_freq?" command
{
    uint32_t sampling_freq_hz = 0;
    //ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);

    *error = CMD_OK;
    memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));
    printf("tx_samp_freq=%u Hz\n\r", sampling_freq_hz);
}

// **************************************************************************
// * @brief Sets the sampling frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_tx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_samp_freq=" command
{
    uint32_t sampling_freq_hz = 0;

    if(param_no >= 1)
    {
        memcpy(&sampling_freq_hz, param, sizeof(sampling_freq_hz));
        //ad9361_set_tx_sampling_freq(ad9361_phy, sampling_freq_hz);
        //ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);

        *error = CMD_OK;
        memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));
        printf("tx_samp_freq=%u Hz\n\r", sampling_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_samp_freq: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current TX RF bandwidth [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_tx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_rf_bandwidth?" command
{
    uint32_t bandwidth_hz = 0;
    //ad9361_get_tx_rf_bandwidth(ad9361_phy, &bandwidth_hz);

    *error = CMD_OK;
    memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));
    printf("tx_rf_bandwidth=%u Hz\n\r", bandwidth_hz);
}

// **************************************************************************
// * @brief Sets the TX RF bandwidth [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_tx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_rf_bandwidth=" command
{
    uint32_t bandwidth_hz = 0;

    if(param_no >= 1)
    {
        memcpy(&bandwidth_hz, param, sizeof(bandwidth_hz));
        //ad9361_set_tx_rf_bandwidth(ad9361_phy, bandwidth_hz);
        //ad9361_get_tx_rf_bandwidth(ad9361_phy, &bandwidth_hz);

        *error = CMD_OK;
        memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));
        printf("tx_rf_bandwidth=%u Hz\n\r", bandwidth_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_rf_bandwidth: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current TX attenuation [mdB].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_tx_attenuation(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx1_attenuation?" command
{
    uint32_t attenuation_mdb = 0;
    //ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb);

    *error = CMD_OK;
    memcpy(response, &attenuation_mdb, sizeof(attenuation_mdb));
    printf("tx_attenuation=%u mdB\n\r", attenuation_mdb);
}

// **************************************************************************
// * @brief Sets the TX attenuation [mdB].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_tx_attenuation(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx1_attenuation=" command
{
    uint32_t attenuation_mdb = 0;

    if(param_no >= 1)
    {
        attenuation_mdb = param[0];
        memcpy(&attenuation_mdb, param, sizeof(attenuation_mdb));
        //ad9361_set_tx_attenuation(ad9361_phy, 0, attenuation_mdb);
        //ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb);

        *error = CMD_OK;
        memcpy(response, &attenuation_mdb, sizeof(attenuation_mdb));
        printf("tx_attenuation=%u mdB\n\r", attenuation_mdb);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_attenuation: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current TX FIR state.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_tx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_fir_en?" command
{
    uint8_t en_dis = 0;

    //ad9361_get_tx_fir_en_dis(ad9361_phy, &en_dis);

    *error = CMD_OK;
    memcpy(response, &en_dis, sizeof(en_dis));
    printf("tx_fir_en=%d\n\r", en_dis);
}

// **************************************************************************
// * @brief Sets the TX FIR state.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_tx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "tx_fir_en=" command
{
    uint8_t en_dis = 0;

    if(param_no >= 1)
    {
        en_dis = param[0];
        memcpy(&en_dis, param, sizeof(en_dis));
        //ad9361_set_tx_fir_en_dis(ad9361_phy, en_dis);
        //ad9361_get_tx_fir_en_dis(ad9361_phy, &en_dis);

        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));
        printf("tx_fir_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_tx_fir_en: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX LO frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_lo_freq?" command
{
    uint64_t lo_freq_hz = 0;
    //ad9361_get_rx_lo_freq(ad9361_phy, &lo_freq_hz);

    lo_freq_hz = static_cast<uint64_t>(m_rx_freq);

    *error = CMD_OK;
    memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));
    printf("rx_lo_freq=%lu Hz\n\r", lo_freq_hz);
}

// **************************************************************************
// * @brief Sets the RX LO frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_lo_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_lo_freq=" command
{
    uint64_t lo_freq_hz = 0;

    if(param_no >= 1)
    {
        memcpy(&lo_freq_hz, param, sizeof(lo_freq_hz));
        m_rx_freq = static_cast<double>(lo_freq_hz);

        double tune_freq = m_adsdr->tune(ad9361_device_t::RX, m_rx_freq);
        lo_freq_hz = static_cast<uint64_t>(tune_freq);

        *error = CMD_OK;
        memcpy(response, &lo_freq_hz, sizeof(lo_freq_hz));
        printf("rx_lo_freq=%lu Hz\n\r", lo_freq_hz);
    }
    else
    {
        printf("ERROR: set_rx_lo_freq: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX sampling frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_samp_freq?" command
{
    uint32_t sampling_freq_hz = 0;
    //ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);

    *error = CMD_OK;
    memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));
    printf("rx_samp_freq=%u Hz\n\r", sampling_freq_hz);
}

// **************************************************************************
// * @brief Sets the RX sampling frequency [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_samp_freq(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_samp_freq=" command
{
    uint32_t sampling_freq_hz = 0;

    if(param_no >= 1)
    {
        sampling_freq_hz = (uint32_t)param[0];
        memcpy(&sampling_freq_hz, param, sizeof(sampling_freq_hz));

        *error = CMD_OK;
        memcpy(response, &sampling_freq_hz, sizeof(sampling_freq_hz));
        printf("rx_samp_freq=%u Hz\n\r", sampling_freq_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: rx_samp_freq: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX RF bandwidth [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_rf_bandwidth?" command
{
    uint32_t bandwidth_hz = static_cast<uint32_t>(m_clock_rate);
    //ad9361_get_rx_rf_bandwidth(ad9361_phy, &bandwidth_hz);

    *error = CMD_OK;
    memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));
    printf("rx_rf_bandwidth=%u Hz\n\r", bandwidth_hz);
}

// **************************************************************************
// * @brief Sets the RX RF bandwidth [Hz].
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_rf_bandwidth(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_rf_bandwidth=" command
{
    uint32_t bandwidth_hz;

    if(param_no >= 1)
    {
        memcpy(&bandwidth_hz, param, sizeof(bandwidth_hz));
        double clock_rate = m_adsdr->set_clock_rate(static_cast<double>(bandwidth_hz));
        bandwidth_hz = static_cast<uint32_t>(clock_rate);

        *error = CMD_OK;
        memcpy(response, &bandwidth_hz, sizeof(bandwidth_hz));
        printf("rx_rf_bandwidth=%u Hz\n\r", bandwidth_hz);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_rf_bandwidth: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX GC mode.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_gc_mode(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_gc_mode?" command
{
    uint8_t gc_mode;
    //ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &gc_mode);

    *error = CMD_OK;
    memcpy(response, &gc_mode, sizeof(gc_mode));
    printf("rx_gc_mode=%d\n\r", gc_mode);
}

// **************************************************************************
// * @brief Sets the RX GC mode.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_gc_mode(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_gc_mode=" command
{
    uint8_t gc_mode;

    if(param_no >= 1)
    {
        memcpy(&gc_mode, param, sizeof(gc_mode));
        *error = CMD_OK;
        memcpy(response, &gc_mode, sizeof(gc_mode));

        printf("rx_gc_mode=%d\n\r", gc_mode);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_gc_mode: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX RF gain.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_rf_gain(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_rf_gain?" command
{
    int32_t gain_db = 0;

    *error = CMD_OK;
    memcpy(response, &gain_db, sizeof(gain_db));
    printf("rx_rf_gain=%d dB\n\r", gain_db);
}

// **************************************************************************
// * @brief Sets the RX RF gain. [dB]
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_rf_gain(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx1_rf_gain=" command
{
    int32_t gain_db = 0;

    if(param_no >= 1)
    {
        memcpy(&gain_db, param, sizeof(gain_db));

        *error = CMD_OK;
        memcpy(response, &gain_db, sizeof(gain_db));

        printf("rx_rf_gain=%d dB\n\r", gain_db);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_rf_gain: invalid parameter!\n\r");
    }
}

// **************************************************************************
// * @brief Gets current RX FIR state.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::get_rx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_fir_en?" command
{
    uint8_t en_dis = 0;

    *error = CMD_OK;
    memcpy(response, &en_dis, sizeof(en_dis));
    printf("rx_fir_en=%d\n\r", en_dis);
}

// **************************************************************************
// * @brief Sets the RX FIR state.
// *
// * @return None.
// **************************************************************************
void ADSDR_impl::set_rx_fir_en(uint64_t *param, char param_no, char* error, uint64_t* response) // "rx_fir_en=" command
{
    uint8_t en_dis;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));
        printf("rx_fir_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_rx_fir_en: invalid parameter!\n\r");
    }
}

void ADSDR_impl::set_datapath_en(uint64_t *param, char param_no, char* error, uint64_t* response)
{
    uint8_t en_dis = 0;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        if(en_dis == 1)
        {
            // Enable FDD
        }
        else
        {
            // Disable
        }

        printf("datapath_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_datapath_en: invalid parameter!\n\r");
    }
}

void ADSDR_impl::get_version(uint64_t *param, char param_no, char* error, uint64_t* response)
{
    uint8_t version[8] = {
            0,//FREESRP_FPGA_VERSION_MAJOR,
            0,//FREESRP_FPGA_VERSION_MINOR,
            1,//FREESRP_FPGA_VERSION_PATCH,
            0,
            0,
            0,
            0,
            0
    };

    *error = CMD_OK;
    memcpy(response, &version, sizeof(uint64_t));
}

void ADSDR_impl::set_loopback_en(uint64_t* param, char param_no, char* error, uint64_t* response)
{
    uint8_t en_dis = 0;

    if(param_no >= 1)
    {
        memcpy(&en_dis, param, sizeof(en_dis));
        *error = CMD_OK;
        memcpy(response, &en_dis, sizeof(en_dis));

        if(en_dis == 1)
        {
            // Enable loopback
            m_adsdr->data_port_loopback(true);
        }
        else
        {
            // Disable
            m_adsdr->data_port_loopback(false);
        }
        printf("loopback_en=%d\n\r", en_dis);
    }
    else
    {
        *error = CMD_INVALID_PARAM;
        printf("ERROR: set_loopback_en: invalid parameter!\n\r");
    }
}

