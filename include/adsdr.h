#ifndef __AD_SDR_H__
#define __AD_SDR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "adsdr_export.h"

#define ADSDR_USB_TIMEOUT 2000

typedef struct adsdr_dev adsdr_dev_t;

typedef struct _AD9361_InitParam AD9361_InitParam_t;
typedef struct _AD9361_RXFIRConfig AD9361_RXFIRConfig_t;
typedef struct _AD9361_TXFIRConfig AD9361_TXFIRConfig_t;


enum adsdr_tuner {
    ADSDR_TUNER_UNKNOWN = 0
};

ADSDR_API enum adsdr_tuner adsdr_get_tuner_type(adsdr_dev_t* dev);

ADSDR_API int adsdr_open(adsdr_dev_t** dev, int index);
ADSDR_API int adsdr_close(adsdr_dev_t* dev);

ADSDR_API int adsdr_find(unsigned int index);
ADSDR_API int adsdr_reset(unsigned int index);
ADSDR_API int adsdr_flash(const char* file_name, unsigned int index);
ADSDR_API unsigned int adsdr_get_device_count();
ADSDR_API const char* adsdr_get_device_name(unsigned int index);
ADSDR_API int  adsdr_get_device_usb_strings(unsigned int index, char* manufact, char* product, char* serial);

ADSDR_API AD9361_InitParam_t* adsdr_get_init_params();
ADSDR_API AD9361_RXFIRConfig_t* adsdr_get_rx_fir_config();
ADSDR_API AD9361_TXFIRConfig_t* adsdr_get_tx_fir_config();

ADSDR_API int adsdr_init(adsdr_dev_t* dev, AD9361_InitParam_t* param, AD9361_RXFIRConfig_t* rx_conf, AD9361_TXFIRConfig_t* tx_conf);

/* streaming functions */
typedef void(*adsdr_read_async_cb_t)(unsigned char* buf, unsigned int len, void* ctx);
ADSDR_API int adsdr_read_async(adsdr_dev_t* dev, adsdr_read_async_cb_t, void* ctx, unsigned int buf_num, unsigned int buf_len);
ADSDR_API int adsdr_cancel_async(adsdr_dev_t* dev);

// Gets current TX LO frequency.
ADSDR_API int adsdr_get_tx_lo_freq(adsdr_dev_t* dev, unsigned long int* lo_freq_hz);
// Sets the TX LO frequency.
ADSDR_API int adsdr_set_tx_lo_freq(adsdr_dev_t* dev, unsigned long int lo_freq_hz);
// Gets current TX sampling frequency.
ADSDR_API int adsdr_get_tx_samp_freq(adsdr_dev_t* dev, unsigned int* sampling_freq_hz);
// Sets the TX sampling frequency.
ADSDR_API int adsdr_set_tx_samp_freq(adsdr_dev_t* dev, unsigned int sampling_freq_hz);
// Gets current TX RF bandwidth.
ADSDR_API int adsdr_get_tx_rf_bandwidth(adsdr_dev_t* dev, unsigned int* bandwidth_hz);
// Sets the TX RF bandwidth.
ADSDR_API int adsdr_set_tx_rf_bandwidth(adsdr_dev_t* dev, unsigned long int bandwidth_hz);
// Gets current TX attenuation.
ADSDR_API int adsdr_get_tx_attenuation(adsdr_dev_t* dev, unsigned char ch, unsigned int* attenuation_mdb);
// Sets the TX attenuation.
ADSDR_API int adsdr_set_tx_attenuation(adsdr_dev_t* dev, unsigned char ch, unsigned int attenuation_mdb);
// Gets current TX FIR state.
ADSDR_API int adsdr_get_tx_fir_en(adsdr_dev_t* dev, unsigned char* en_dis);
// Sets the TX FIR state.
ADSDR_API int adsdr_set_tx_fir_en(adsdr_dev_t* dev, unsigned char en_dis);

// Gets current RX LO frequency.
ADSDR_API int adsdr_get_rx_lo_freq(adsdr_dev_t* dev, unsigned long int* lo_freq_hz);
// Sets the RX LO frequency.
ADSDR_API int adsdr_set_rx_lo_freq(adsdr_dev_t* dev, unsigned long int lo_freq_hz);
// Gets current RX sampling frequency.
ADSDR_API int adsdr_get_rx_samp_freq(adsdr_dev_t* dev, unsigned int* sampling_freq_hz);
// Sets the RX sampling frequency.
ADSDR_API int adsdr_set_rx_samp_freq(adsdr_dev_t* dev, unsigned int sampling_freq_hz);
// Gets current RX RF bandwidth.
ADSDR_API int adsdr_get_rx_rf_bandwidth(adsdr_dev_t* dev, unsigned int* bandwidth_hz);
// Sets the RX RF bandwidth.
ADSDR_API int adsdr_set_rx_rf_bandwidth(adsdr_dev_t* dev, unsigned int bandwidth_hz);
// Gets current RX1 GC mode.
ADSDR_API int adsdr_get_rx_gc_mode(adsdr_dev_t* dev, unsigned char ch, unsigned char* gc_mode);
// Sets the RX GC mode.
ADSDR_API int adsdr_set_rx_gc_mode(adsdr_dev_t* dev, unsigned char ch, unsigned char gc_mode);
// Gets current RX RF gain.
ADSDR_API int adsdr_get_rx_rf_gain(adsdr_dev_t* dev, unsigned char ch, int* gain_db);
// Sets the RX RF gain.
ADSDR_API int adsdr_set_rx_rf_gain(adsdr_dev_t* dev, unsigned char ch, int gain_db);
// Gets current RX FIR state.
ADSDR_API int adsdr_get_rx_fir_en(adsdr_dev_t* dev, unsigned char* en_dis);
// Sets the RX FIR state.
ADSDR_API int adsdr_set_rx_fir_en(adsdr_dev_t* dev, unsigned char en_dis);

// Enables/disables the datapath. (Puts AD9364 into FDD state/alert state and notifies the rest of the FPGA system) */
ADSDR_API int adsdr_set_datapath_en(adsdr_dev_t* dev, unsigned char mode);
// Enables/disables the AD9364's loopback BIST mode
ADSDR_API int adsdr_set_loopback_en(adsdr_dev_t* dev, int mode);


#ifdef __cplusplus
}
#endif


#endif // __AD_SDR_H__


