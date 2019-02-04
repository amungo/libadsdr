
#include "ad9361_api.h"
#include "adsdr.h"
#include "parameters.h"
#include "platform.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "fx3deverr.h"
#include <libusb.h>


#define GET_LSW(v) ((unsigned short)((v) & 0xFFFF))
#define GET_MSW(v) ((unsigned short)((v) >> 16))
#define MAX_WRITE_SIZE (2 * 1024)

#define DEFAULT_BUF_NUMBER	15
#define DEFAULT_BUF_LENGTH	(16 * 32 * 512)

#define CY_SIGN_BYTE_0 0x43 // Cypress firmware file format specific: first byte in firmware file
#define CY_SIGN_BYTE_1 0x59 // Cypress firmware file format specific: second byte in firmware file

#define ADSR_RX_IN      0x81
#define ADSR_DEBUG_IN   0x82
#define BULK_TIMEOUT    1000

enum adsdr_async_status {
    ADSDR_INACTIVE = 0,
    ADSDR_CANCELING,
    ADSDR_RUNNING
};

typedef struct adsdr_description {
    uint16_t vid;
    uint16_t pid;
    const char* name;
} adsdr_description_t;

static adsdr_description_t adsdr_devices[] = {
    {ADSDR_VENDOR_ID, ADSDR_BOOT_ID, "Generic FX3 boot"},
    {ADSDR_VENDOR_ID, ADSDR_PRODUCT_ID, "Generic ADSDR "},
};


void print_ensm_state(struct ad9361_rf_phy* phy);


#if 0
//@camry =====================================
#include <pthread.h>

uint32_t g_data_count;
uint32_t g_exit_timer_thread = 0;
pthread_t tid;

void* timer_proc(void* arg)
{
    fprintf(stderr,"Start timer speed procedure \n");
    while(!g_exit_timer_thread) {
        usleep(1000*1000);

        fprintf(stderr,"--- samperate: %f MB/s \n", (float)g_data_count / (float)(1024*1024));
        g_data_count = 0;
    }
}

// ==========================================
#endif


struct adsdr_dev {
    struct ad9361_rf_phy* ad9361dev;
    libusb_context* ctx;
    libusb_device_handle* handle;

    uint32_t xfer_buf_num;
    uint32_t xfer_buf_len;
    struct libusb_transfer **xfer;
    unsigned char **xfer_buf;
    adsdr_read_async_cb_t cb;
    void* cb_ctx;
    enum adsdr_async_status async_status;
    int32_t async_cancel;
    int use_zerocopy;
    /* tuner context */
    enum adsdr_tuner tuner_type;

    /* status */
    int dev_lost;
    int driver_active;
    unsigned int xfer_errors;
};

static AD9361_InitParam_t s_ad_default_param = {
        /* Device selection */
        ID_AD9361,	// dev_sel
        /* Identification number */
        0,		//id_no
        /* Reference Clock */
        40000000UL,	//reference_clk_rate
        /* Base Configuration */
        1,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
        1,		//one_rx_one_tx_mode_use_rx_num *** adi,1rx-1tx-mode-use-rx-num
        1,		//one_rx_one_tx_mode_use_tx_num *** adi,1rx-1tx-mode-use-tx-num
        1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
        0,		//frequency_division_duplex_independent_mode_enable *** adi,frequency-division-duplex-independent-mode-enable
        0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
        0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
        0,		//tx_fastlock_delay_ns *** adi,tx-fastlock-delay-ns
        0,		//rx_fastlock_delay_ns *** adi,rx-fastlock-delay-ns
        0,		//rx_fastlock_pincontrol_enable *** adi,rx-fastlock-pincontrol-enable
        0,		//tx_fastlock_pincontrol_enable *** adi,tx-fastlock-pincontrol-enable
        0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
        0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
        5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
        6,		//dc_offset_attenuation_high_range *** adi,dc-offset-attenuation-high-range
        5,		//dc_offset_attenuation_low_range *** adi,dc-offset-attenuation-low-range
        0x28,	//dc_offset_count_high_range *** adi,dc-offset-count-high-range
        0x32,	//dc_offset_count_low_range *** adi,dc-offset-count-low-range
        0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
        MAX_SYNTH_FREF,	//trx_synthesizer_target_fref_overwrite_hz *** adi,trx-synthesizer-target-fref-overwrite-hz
        0,		// qec_tracking_slow_mode_enable *** adi,qec-tracking-slow-mode-enable
        /* ENSM Control */
        0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
        0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
        /* LO Control */
        2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
        2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
        1,				//tx_lo_powerdown_managed_enable *** adi,tx-lo-powerdown-managed-enable
        /* Rate & BW Control */
        {983040000, 245760000, 122880000, 61440000, 30720000, 30720000},// rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
        {983040000, 122880000, 122880000, 61440000, 30720000, 30720000},// tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies

        18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
        18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
        /* RF Port Control */
        1,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
        0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
        /* TX Attenuation Control */
        10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
        0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
        /* Reference Clock Control */
        0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
        {8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
        CLKOUT_DISABLE,	//clk_output_mode_select *** adi,clk-output-mode-select
        /* Gain Control */
        2,		//gc_rx1_mode *** adi,gc-rx1-mode
        2,		//gc_rx2_mode *** adi,gc-rx2-mode
        58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
        4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
        47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
        8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
        0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
        800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
        704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
        24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
        15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
        /* Gain MGC Control */
        2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
        2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
        0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
        0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
        0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
        /* Gain AGC Control */
        10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
        2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
        0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
        10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
        4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
        3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
        1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
        0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
        0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
        10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
        1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
        12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
        1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
        10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
        2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
        10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
        5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
        2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
        18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
        2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
        1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
        0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
        /* Fast AGC */
        64,		//fagc_dec_pow_measuremnt_duration ***  adi,fagc-dec-pow-measurement-duration
        260,	//fagc_state_wait_time_ns ***  adi,fagc-state-wait-time-ns
        /* Fast AGC - Low Power */
        0,		//fagc_allow_agc_gain_increase ***  adi,fagc-allow-agc-gain-increase-enable
        5,		//fagc_lp_thresh_increment_time ***  adi,fagc-lp-thresh-increment-time
        1,		//fagc_lp_thresh_increment_steps ***  adi,fagc-lp-thresh-increment-steps
        /* Fast AGC - Lock Level (Lock Level is set via slow AGC inner high threshold) */
        1,		//fagc_lock_level_lmt_gain_increase_en ***  adi,fagc-lock-level-lmt-gain-increase-enable
        5,		//fagc_lock_level_gain_increase_upper_limit ***  adi,fagc-lock-level-gain-increase-upper-limit
        /* Fast AGC - Peak Detectors and Final Settling */
        1,		//fagc_lpf_final_settling_steps ***  adi,fagc-lpf-final-settling-steps
        1,		//fagc_lmt_final_settling_steps ***  adi,fagc-lmt-final-settling-steps
        3,		//fagc_final_overrange_count ***  adi,fagc-final-overrange-count
        /* Fast AGC - Final Power Test */
        0,		//fagc_gain_increase_after_gain_lock_en ***  adi,fagc-gain-increase-after-gain-lock-enable
        /* Fast AGC - Unlocking the Gain */
        0,		//fagc_gain_index_type_after_exit_rx_mode ***  adi,fagc-gain-index-type-after-exit-rx-mode
        1,		//fagc_use_last_lock_level_for_set_gain_en ***  adi,fagc-use-last-lock-level-for-set-gain-enable
        1,		//fagc_rst_gla_stronger_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable
        5,		//fagc_optimized_gain_offset ***  adi,fagc-optimized-gain-offset
        10,		//fagc_rst_gla_stronger_sig_thresh_above_ll ***  adi,fagc-rst-gla-stronger-sig-thresh-above-ll
        1,		//fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable
        1,		//fagc_rst_gla_engergy_lost_goto_optim_gain_en ***  adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable
        10,		//fagc_rst_gla_engergy_lost_sig_thresh_below_ll ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll
        8,		//fagc_energy_lost_stronger_sig_gain_lock_exit_cnt ***  adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt
        1,		//fagc_rst_gla_large_adc_overload_en ***  adi,fagc-rst-gla-large-adc-overload-enable
        1,		//fagc_rst_gla_large_lmt_overload_en ***  adi,fagc-rst-gla-large-lmt-overload-enable
        0,		//fagc_rst_gla_en_agc_pulled_high_en ***  adi,fagc-rst-gla-en-agc-pulled-high-enable
        0,		//fagc_rst_gla_if_en_agc_pulled_high_mode ***  adi,fagc-rst-gla-if-en-agc-pulled-high-mode
        64,		//fagc_power_measurement_duration_in_state5 ***  adi,fagc-power-measurement-duration-in-state5
        /* RSSI Control */
        1,		//rssi_delay *** adi,rssi-delay
        1000,	//rssi_duration *** adi,rssi-duration
        3,		//rssi_restart_mode *** adi,rssi-restart-mode
        0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
        1,		//rssi_wait *** adi,rssi-wait
        /* Aux ADC Control */
        256,	//aux_adc_decimation *** adi,aux-adc-decimation
        40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
        /* AuxDAC Control */
        1,		//aux_dac_manual_mode_enable ***  adi,aux-dac-manual-mode-enable
        0,		//aux_dac1_default_value_mV ***  adi,aux-dac1-default-value-mV
        0,		//aux_dac1_active_in_rx_enable ***  adi,aux-dac1-active-in-rx-enable
        0,		//aux_dac1_active_in_tx_enable ***  adi,aux-dac1-active-in-tx-enable
        0,		//aux_dac1_active_in_alert_enable ***  adi,aux-dac1-active-in-alert-enable
        0,		//aux_dac1_rx_delay_us ***  adi,aux-dac1-rx-delay-us
        0,		//aux_dac1_tx_delay_us ***  adi,aux-dac1-tx-delay-us
        0,		//aux_dac2_default_value_mV ***  adi,aux-dac2-default-value-mV
        0,		//aux_dac2_active_in_rx_enable ***  adi,aux-dac2-active-in-rx-enable
        0,		//aux_dac2_active_in_tx_enable ***  adi,aux-dac2-active-in-tx-enable
        0,		//aux_dac2_active_in_alert_enable ***  adi,aux-dac2-active-in-alert-enable
        0,		//aux_dac2_rx_delay_us ***  adi,aux-dac2-rx-delay-us
        0,		//aux_dac2_tx_delay_us ***  adi,aux-dac2-tx-delay-us
        /* Temperature Sensor Control */
        256,	//temp_sense_decimation *** adi,temp-sense-decimation
        1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
        (int8_t)0xCE,	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
        1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
        /* Control Out Setup */
        0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
        0,		//ctrl_outs_index *** adi,ctrl-outs-index
        /* External LNA Control */
        0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
        0,		//elna_gain_mdB *** adi,elna-gain-mdB
        0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
        0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
        0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
        0,		//elna_gaintable_all_index_enable *** adi,elna-gaintable-all-index-enable
        /* Digital Interface Control */
        0,		//digital_interface_tune_skip_mode *** adi,digital-interface-tune-skip-mode
        0,		//digital_interface_tune_fir_disable *** adi,digital-interface-tune-fir-disable
        1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
        1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
        0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
        0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
        1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
        0,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
        0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
        0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
        0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
        0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
        0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
        0,		//swap_ports_enable *** adi,swap-ports-enable
        1,		//single_data_rate_enable *** adi,single-data-rate-enable
        0,		//lvds_mode_enable *** adi,lvds-mode-enable
        0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
        0,		//single_port_mode_enable *** adi,single-port-mode-enable
        1,		//full_port_enable *** adi,full-port-enable
        0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
        0,		//delay_rx_data *** adi,delay-rx-data
        0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
        4,		//rx_data_delay *** adi,rx-data-delay
        7,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
        0,		//tx_data_delay *** adi,tx-data-delay
        150,	//lvds_bias_mV *** adi,lvds-bias-mV
        1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
        0,		//rx1rx2_phase_inversion_en *** adi,rx1-rx2-phase-inversion-enable
        0xFF,	//lvds_invert1_control *** adi,lvds-invert1-control
        0x0F,	//lvds_invert2_control *** adi,lvds-invert2-control
        /* GPO Control */
        0,		//gpo0_inactive_state_high_enable *** adi,gpo0-inactive-state-high-enable
        0,		//gpo1_inactive_state_high_enable *** adi,gpo1-inactive-state-high-enable
        0,		//gpo2_inactive_state_high_enable *** adi,gpo2-inactive-state-high-enable
        0,		//gpo3_inactive_state_high_enable *** adi,gpo3-inactive-state-high-enable
        0,		//gpo0_slave_rx_enable *** adi,gpo0-slave-rx-enable
        0,		//gpo0_slave_tx_enable *** adi,gpo0-slave-tx-enable
        0,		//gpo1_slave_rx_enable *** adi,gpo1-slave-rx-enable
        0,		//gpo1_slave_tx_enable *** adi,gpo1-slave-tx-enable
        0,		//gpo2_slave_rx_enable *** adi,gpo2-slave-rx-enable
        0,		//gpo2_slave_tx_enable *** adi,gpo2-slave-tx-enable
        0,		//gpo3_slave_rx_enable *** adi,gpo3-slave-rx-enable
        0,		//gpo3_slave_tx_enable *** adi,gpo3-slave-tx-enable
        0,		//gpo0_rx_delay_us *** adi,gpo0-rx-delay-us
        0,		//gpo0_tx_delay_us *** adi,gpo0-tx-delay-us
        0,		//gpo1_rx_delay_us *** adi,gpo1-rx-delay-us
        0,		//gpo1_tx_delay_us *** adi,gpo1-tx-delay-us
        0,		//gpo2_rx_delay_us *** adi,gpo2-rx-delay-us
        0,		//gpo2_tx_delay_us *** adi,gpo2-tx-delay-us
        0,		//gpo3_rx_delay_us *** adi,gpo3-rx-delay-us
        0,		//gpo3_tx_delay_us *** adi,gpo3-tx-delay-us
        /* Tx Monitor Control */
        37000,	//low_high_gain_threshold_mdB *** adi,txmon-low-high-thresh
        0,		//low_gain_dB *** adi,txmon-low-gain
        24,		//high_gain_dB *** adi,txmon-high-gain
        0,		//tx_mon_track_en *** adi,txmon-dc-tracking-enable
        0,		//one_shot_mode_en *** adi,txmon-one-shot-mode-enable
        511,	//tx_mon_delay *** adi,txmon-delay
        8192,	//tx_mon_duration *** adi,txmon-duration
        2,		//tx1_mon_front_end_gain *** adi,txmon-1-front-end-gain
        2,		//tx2_mon_front_end_gain *** adi,txmon-2-front-end-gain
        48,		//tx1_mon_lo_cm *** adi,txmon-1-lo-cm
        48,		//tx2_mon_lo_cm *** adi,txmon-2-lo-cm
        /* GPIO definitions */
        GPIO_RESET_PIN,		//gpio_resetb *** reset-gpios
        /* MCS Sync */
        -1,		//gpio_sync *** sync-gpios
        -1,		//gpio_cal_sw1 *** cal-sw1-gpios
        -1,		//gpio_cal_sw2 *** cal-sw2-gpios
        /* External LO clocks */
        NULL,	//(*ad9361_rfpll_ext_recalc_rate)()
        NULL,	//(*ad9361_rfpll_ext_round_rate)()
        NULL,	//(*ad9361_rfpll_ext_set_rate)()
        1       // Only rx mode.
    };

static AD9361_RXFIRConfig_t s_rx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
        3, // rx
        0, // rx_gain
        1, // rx_dec
        {-4, -6, -37, 35, 186, 86, -284, -315,
         107, 219, -4, 271, 558, -307, -1182, -356,
         658, 157, 207, 1648, 790, -2525, -2553, 748,
         865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
         14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
         748, -2553, -2525, 790, 1648, 207, 157, 658,
         -356, -1182, -307, 558, 271, -4, 219, 107,
         -315, -284, 86, 186, 35, -37, -6, -4,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0}, // rx_coef[128]
        64, // rx_coef_size
        {0, 0, 0, 0, 0, 0}, //rx_path_clks[6]
        0 // rx_bandwidth
    };

static AD9361_TXFIRConfig_t s_tx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
        3, // tx
        -6, // tx_gain
        1, // tx_int
        {-4, -6, -37, 35, 186, 86, -284, -315,
         107, 219, -4, 271, 558, -307, -1182, -356,
         658, 157, 207, 1648, 790, -2525, -2553, 748,
         865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
         14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
         748, -2553, -2525, 790, 1648, 207, 157, 658,
         -356, -1182, -307, 558, 271, -4, 219, 107,
         -315, -284, 86, 186, 35, -37, -6, -4,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0}, // tx_coef[128]
        64, // tx_coef_size
        {0, 0, 0, 0, 0, 0}, // tx_path_clks[6]
        0 // tx_bandwidth
    };



ADSDR_API AD9361_InitParam_t* adsdr_get_init_params()
{
    return &s_ad_default_param;
}
ADSDR_API AD9361_RXFIRConfig_t* adsdr_get_rx_fir_config()
{
    return &s_rx_fir_config;
}
ADSDR_API AD9361_TXFIRConfig_t* adsdr_get_tx_fir_config()
{
    return &s_tx_fir_config;
}

static adsdr_description_t* find_adsdr_device(uint16_t vid, uint16_t pid)
{
    unsigned int i;
    adsdr_description_t* device = NULL;

    for(i = 0; i < sizeof(adsdr_devices)/sizeof(adsdr_description_t); i++) {
        if(adsdr_devices[i].vid == vid && adsdr_devices[i].pid == pid) {
            device = &adsdr_devices[i];
            break;
        }
    }

    return device;
}

int ram_write(libusb_device_handle* fx3_handle, uint8_t* buf, uint32_t ramAddress, int32_t len)
{
    int index = 0;

    while(len > 0)
    {
        int size = (len > MAX_WRITE_SIZE) ? MAX_WRITE_SIZE : len;
        int r = libusb_control_transfer(fx3_handle, 0x40, 0xA0, GET_LSW(ramAddress), GET_MSW(ramAddress), &buf[index], size, ADSDR_USB_TIMEOUT);
        if(r != size)
        {
            fprintf(stderr, "FX3 firmware write via libusb control transfer failed: %d\n", r);
            return FX3_ERR_CTRL_TX_FAIL;
        }

        ramAddress += size;
        index += size;
        len -= size;
    }
}


uint32_t adsdr_get_device_count()
{
    int i;
    int ret;
    libusb_context* ctx;
    libusb_device** devs;
    uint32_t device_count = 0;
    struct libusb_device_descriptor desc;
    int num_devs;

    ret = libusb_init(&ctx);
    if(ret < 0)
    {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return ret;
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if(num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d \n", num_devs);
        libusb_exit(ctx);
        return num_devs;
    }

    for(i = 0; i < num_devs; i++) {
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0) {
            fprintf(stderr, "libusb error getting device descriptor: error %d \n", ret);
            libusb_free_device_list(devs, 1);
            libusb_exit(ctx);
            return ret;
        }

        if(find_adsdr_device(desc.idVendor, desc.idProduct))
            device_count++;
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return device_count;
}

const char* adsdr_get_device_name(uint32_t index)
{
    int i;
    int ret;
    libusb_context* ctx;
    libusb_device** devs;
    uint32_t device_count = 0;
    struct libusb_device_descriptor desc;
    adsdr_description_t* device = NULL;
    int num_devs;

    ret = libusb_init(&ctx);
    if(ret < 0)
    {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return "";
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if(num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d \n", num_devs);
        libusb_exit(ctx);
        return "";
    }

    for(i = 0; i < num_devs; i++) {
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0) {
            fprintf(stderr, "libusb error getting device descriptor: error %d \n", ret);
            libusb_free_device_list(devs, 1);
            libusb_exit(ctx);
            return "";
        }

        device = find_adsdr_device(desc.idVendor, desc.idProduct);
        if(device) {
            device_count++;

            if(index == device_count - 1)
                break;
        }
        device = NULL;
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
    if(device)
        return device->name;
    else
        return "";
}

int32_t adsdr_flash(const char* file_name, uint32_t index)
{
    libusb_context* ctx = 0;
    libusb_device** devs;
    libusb_device_handle* handle = 0;
    libusb_device* device = NULL;
    struct libusb_device_descriptor desc;
    int32_t device_count = 0;
    int32_t ret = -1;
    int32_t i;
    int num_devs;

    FILE* fwfile;
    uint32_t fw_size8;
    uint8_t* fw_buf;
    uint32_t fw_index = 4;
    uint32_t checksum = 0;
    uint32_t* data_p;
    int32_t address;
    int32_t length;

    ret = libusb_init(&ctx);
    if(ret < 0) {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return ret;
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if (num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d \n", num_devs);
        libusb_exit(ctx);
        return num_devs;
    }

    for(i = 0; i < num_devs; i++)
    {
        device = devs[i];
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if (ret < 0) {
            fprintf(stderr, "libusb error getting device descriptor: error %d \n", ret);
            libusb_free_device_list(devs, 1);
            libusb_exit(ctx);
            return ret;
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_BOOT_ID)
            device_count++;

        if(index == device_count - 1)
            break;

        device = NULL;
    }

    if(!device) {
        fprintf(stderr, "ADSDR device not found!\n");
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return FX3_ERR_NO_DEVICE_FOUND;
    }

    ret = libusb_open(device, &handle);
    if(ret < 0) {
        libusb_free_device_list(devs, 1);
        fprintf(stderr, "usb open error %d\n", ret);
        if(ret == LIBUSB_ERROR_ACCESS)
            fprintf(stderr, "Please fix the device permissions, by installing the udev rules file adsdr.rules\n");
        libusb_exit(ctx);
        return ret;
    }

    fwfile = fopen(file_name, "rb");

    if(!fwfile) {
        fprintf(stderr, "Error open firmware file: %s\n", file_name);
        return FX3_ERR_FIRMWARE_FILE_IO_ERROR;
    }

    // Find file size
    fseek(fwfile, 0, SEEK_END);
    fw_size8 = ftell(fwfile);
    fseek(fwfile, 0, SEEK_SET);

    // Read firmware
    fw_buf = malloc(fw_size8);
    fread(fw_buf, 1, fw_size8, fwfile);
    fclose(fwfile);

    // Check Cypress signature
    if(fw_buf[0] != CY_SIGN_BYTE_0 || fw_buf[1] != CY_SIGN_BYTE_1) {
        fprintf(stderr, "bad Cypress signarure: 0x%02X 0x%02X\n", fw_buf[0], fw_buf[1]);
        free(fw_buf);
        return FX3_ERR_FIRMWARE_FILE_CORRUPTED;
    }

    // Write firmware to RAM
    while(fw_index < fw_size8)
    {
        data_p = (uint32_t*)(fw_buf + fw_index);
        length = data_p[0];
        address = data_p[1];
        if(length != 0)
        {
            for(i = 0; i < length; i++) {
                checksum += data_p[2 + i];
            }
            ram_write(handle, fw_buf + fw_index + 8, address, length*4);
        }
        else
        {
            if(checksum != data_p[2]) {
                fprintf(stderr, "Checksum error in firmware binary\n");
                free(fw_buf);
                return FX3_ERR_FIRMWARE_FILE_CORRUPTED;
            }
            libusb_control_transfer(handle, 0x40, 0xA0, GET_LSW(address), GET_MSW(address), 0, 0, ADSDR_USB_TIMEOUT);
            break;
        }
        fw_index += (8 + length * 4);
    }

    // Free memory buffer
    free(fw_buf);
    libusb_free_device_list(devs, 1);

    libusb_close(handle);
    libusb_exit(ctx);

    return FX3_ERR_OK;
}


ADSDR_API int32_t adsdr_find(uint32_t index)
{
    libusb_context* ctx = 0;
    libusb_device** devs;
    struct libusb_device_descriptor desc;
    int32_t device_count = 0;

    int32_t ret = -1;
    int32_t i;
    int32_t num_devs;

    ret = libusb_init(&ctx);
    if(ret < 0) {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return ret;
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if(num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d\n", num_devs);
        libusb_exit(ctx);
        return num_devs;
    }

    for(i = 0; i < num_devs; i++)
    {
        libusb_get_device_descriptor(devs[i], &desc);

        if(find_adsdr_device(desc.idVendor, desc.idProduct))
            device_count++;

        if(index == device_count - 1) {
            ret = desc.idProduct;
            break;
        }
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return ret;
}

ADSDR_API int32_t adsdr_reset(uint32_t index)
{
    libusb_context* ctx = 0;
    libusb_device** devs;
    libusb_device_handle* handle = 0;
    libusb_device* device = NULL;
    struct libusb_device_descriptor desc;
    int32_t device_count = 0;

    int32_t ret = -1;
    int32_t i;
    int num_devs;

    ret = libusb_init(&ctx);
    if(ret < 0) {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return ret;
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if (num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d \n", num_devs);
        libusb_exit(ctx);
        return num_devs;
    }

    for(i = 0; i < num_devs; i++)
    {
        device = devs[i];
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if (ret < 0) {
            fprintf(stderr, "libusb error getting device descriptor: error %d \n", ret);
            libusb_free_device_list(devs, 1);
            libusb_exit(ctx);
            return ret;
        }

        if(desc.idVendor == ADSDR_VENDOR_ID && desc.idProduct == ADSDR_PRODUCT_ID)
            device_count++;

        if(index == device_count - 1)
            break;

        device = NULL;
    }

    if(!device) {
        fprintf(stderr, "ADSDR device not found!\n");
        libusb_free_device_list(devs, 1);
        libusb_exit(ctx);
        return FX3_ERR_NO_DEVICE_FOUND;
    }

    ret = libusb_open(device, &handle);
    if(ret < 0) {
        libusb_free_device_list(devs, 1);
        fprintf(stderr, "usb open error %d\n", ret);
        if(ret == LIBUSB_ERROR_ACCESS)
            fprintf(stderr, "Please fix the device permissions, by installing the udev rules file adsdr.rules\n");
        libusb_exit(ctx);
        return ret;
    }

    uint8_t data[3] = {0xFF, 0xFF, 0xFF};
    ret = libusb_control_transfer(handle, LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, 0xB3, 0, 0, data, 3, ADSDR_USB_TIMEOUT);
    if(ret < 0) {
        fprintf(stderr, "libusb error resetting device: error %d \n", ret);
    }

    libusb_close(handle);
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return ret;

}

static int32_t _device_start()
{
    uint8_t buf[1] = {0};
    int ret = txControlToDevice(buf, 1, DEVICE_START, 0, 0);
    if(ret < 0)
        fprintf(stderr, "_device start transfer error: %d \n", ret);

    return ret;
}

static int32_t _device_stop()
{
    uint8_t buf[1] = {0};
    int ret = txControlToDevice(buf, 1, DEVICE_STOP, 0, 0);
    if(ret < 0)
        fprintf(stderr, "_device stop transfer error: %d \n", ret);

    return ret;
}

int32_t adsdr_get_usb_strings(adsdr_dev_t* dev, char* manufact, char* product, char* serial)
{
    struct libusb_device_descriptor ddesc;
    libusb_device* device = NULL;
    const int buf_max = 256;
    int r = 0;

    if(!dev || !dev->handle)
        return -1;

    device = libusb_get_device(dev->handle);

    r = libusb_get_device_descriptor(device, &ddesc);
    if(r < 0)
        return -1;

    if(manufact) {
        memset(manufact, 0, buf_max);
        libusb_get_string_descriptor_ascii(dev->handle, ddesc.iManufacturer, (unsigned char*)manufact, buf_max);
    }

    if(product) {
        memset(product, 0, buf_max);
        libusb_get_string_descriptor_ascii(dev->handle, ddesc.iProduct, (unsigned char*)product, buf_max);
    }

    if(serial) {
        memset(serial, 0, buf_max);
        libusb_get_string_descriptor_ascii(dev->handle, ddesc.iSerialNumber, (unsigned char*)serial, buf_max);
    }

    return FX3_ERR_OK;
}


int32_t adsdr_get_device_usb_strings(uint32_t index, char* manufact, char* product, char* serial)
{
    int i;
    int ret;
    libusb_context* ctx;
    libusb_device** devs;
    uint32_t device_count = 0;
    struct libusb_device_descriptor desc;
    adsdr_description_t* device = NULL;
    adsdr_dev_t devt;
    int num_devs;

    ret = libusb_init(&ctx);
    if(ret < 0)
    {
        fprintf(stderr, "libusb init error: %d \n", ret);
        return ret;
    }

    num_devs = libusb_get_device_list(ctx, &devs);
    if(num_devs < 0) {
        fprintf(stderr, "libusb device list retrieval error: %d \n", num_devs);
        libusb_exit(ctx);
        return num_devs;
    }

    for(i = 0; i < num_devs; i++) {
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if(ret < 0) {
            fprintf(stderr, "libusb error getting device descriptor: error %d \n", ret);
            libusb_free_device_list(devs, 1);
            libusb_exit(ctx);
            return ret;
        }

        device = find_adsdr_device(desc.idVendor, desc.idProduct);
        if(device) {
            device_count++;

            if(index == device_count - 1) {
                ret = libusb_open(devs[i], &devt.handle);
                if(ret == FX3_ERR_OK) {
                    ret = adsdr_get_usb_strings(&devt, manufact, product, serial);
                    libusb_close(devt.handle);
                }
                break;
            }
        }
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return ret;
}


static void LIBUSB_CALL _libusb_callback(struct libusb_transfer* xfer)
{
    adsdr_dev_t* dev = (adsdr_dev_t*)xfer->user_data;

    switch(xfer->status)
    {
    case LIBUSB_TRANSFER_COMPLETED:
        if(dev->cb) {
            dev->cb(xfer->buffer, xfer->actual_length, dev->cb_ctx);
#if 0
            g_data_count += xfer->actual_length; //@camry
#endif
            // resubmit transfer
            libusb_submit_transfer(xfer);
        }
        break;
    case LIBUSB_TRANSFER_ERROR:
        dev->xfer_errors++;
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        dev->dev_lost = 1;
        adsdr_cancel_async(dev);
        fprintf(stderr, "cb transfer status (no device): %d, cancelling...\n", xfer->status);
        break;
    case LIBUSB_TRANSFER_TIMED_OUT:
        fprintf(stderr, "cb transfer timeout, cancelling...\n");
        adsdr_cancel_async(dev);
        break;
    case LIBUSB_TRANSFER_CANCELLED:
        break;
    default:
        break;
    }

}

int32_t adsdr_open(adsdr_dev_t** out_dev, int32_t index)
{
    int r;
    int i;
    libusb_device** list;
    adsdr_dev_t* dev = NULL;
    libusb_device* device = NULL;
    int32_t device_count = 0;
    struct libusb_device_descriptor ddesc;
    ssize_t cnt;

    dev = malloc(sizeof(adsdr_dev_t));
    if(dev == NULL)
        return -ENOMEM;

    memset(dev, 0, sizeof(adsdr_dev_t));

    r = libusb_init(&dev->ctx);
    if(r < 0) {
        fprintf(stderr, "libusb_init error %d\n", r);
        free(dev);
        return -1;
    }

    // Set verbosity level
    libusb_set_debug(dev->ctx, 3);

    // Retrieve device list
    cnt = libusb_get_device_list(dev->ctx, &list);
    device_count = 0;

    for(i = 0; i < cnt; i++)
    {
        device = list[i];
        libusb_get_device_descriptor(list[i], &ddesc);

        if(find_adsdr_device(ddesc.idVendor, ddesc.idProduct))
            device_count++;

        if(index == device_count - 1)
            break;

        device = NULL;
    }

    if(!device) {
        fprintf(stderr, "ADSDR device not found!\n");
        r = FX3_ERR_NO_DEVICE_FOUND;
        goto err;
    }

    r = libusb_open(device, &dev->handle);
    if(r < 0) {
        libusb_free_device_list(list, 1);
        fprintf(stderr, "usb open error %d\n", r);
        if(r == LIBUSB_ERROR_ACCESS)
            fprintf(stderr, "Please fix the device permissions, by installing the udev rules file adsdr.rules\n");
        goto err;
    }

    libusb_free_device_list(list, 1);

    // Found a ADSDR device and opened it. Now claim its interface (ID 0).
    r = libusb_claim_interface(dev->handle, 0);
    if(r != FX3_ERR_OK) {
        fprintf(stderr, "libusb claim interface error %d\n", r);
        goto err;
    }

    dev->tuner_type = ADSDR_TUNER_UNKNOWN;

    set_libusb_params(dev->ctx, dev->handle);

    *out_dev = dev;

    return r;

err:
    if(dev) {
        if(dev->ctx)
            libusb_exit(dev->ctx);
        free(dev);
    }

    return r;
}

int32_t adsdr_close(adsdr_dev_t* dev)
{
    if(!dev)
        return FX3_ERR_BAD_DEVICE;

    set_libusb_params(0, 0);

    libusb_release_interface(dev->handle, 0);

    libusb_close(dev->handle);
    libusb_exit(dev->ctx);

    free(dev);

    return 0;
}

int32_t adsdr_init(adsdr_dev_t* dev, AD9361_InitParam_t* ad_default_param, AD9361_RXFIRConfig_t* rx_fir_conf, AD9361_TXFIRConfig_t* tx_fir_conf)
{
    int32_t ret;
    int32_t gain;

    ret = ad9361_init(&dev->ad9361dev, ad_default_param);
    if(ret != 0) {
        fprintf(stderr, "Error ad9361_init\n");
        return ret;
    }


    ret = ad9361_set_rx_fir_config(dev->ad9361dev, *rx_fir_conf);
    if(ret != 0) {
        fprintf(stderr, "Error ad9361_set_rx_fir_config\n");
        return ret;
    }

#if 0
    ret = ad9361_set_tx_fir_config(dev->ad9361dev, *tx_fir_conf);
    if(ret != 0) {
        fprintf(stderr, "Error ad9361_set_tx_fir_config\n");
        return ret;
    }
#endif

    ret = ad9361_set_no_ch_mode(dev->ad9361dev, MODE_1x1);
    if(ret != 0) {
        fprintf(stderr, "Error ad9361_set_no_ch_mode\n");
        return ret;
    }

    print_ensm_state(dev->ad9361dev);
    ret = ad9361_set_en_state_machine_mode(dev->ad9361dev, ENSM_MODE_WAIT);
    if(ret != 0) {
        fprintf(stderr, "Error ad9361_set_en_state_machine_mode\n");
        return ret;
    }

    print_ensm_state(dev->ad9361dev);

#if 0
    //@camry ==================================================
    ret = ad9361_get_rx_rf_gain(dev->ad9361dev, RX1, &gain);
    if(ret == 0)
        fprintf(stderr, "=== Get gain %d.  ===\n", gain);
    else
        fprintf(stderr, "=== Error get gain. Err: %d.  ===\n", ret);


    ret = ad9361_set_rx_gain_control_mode(dev->ad9361dev, RX1, RF_GAIN_MGC);
    if(ret < 0)
        fprintf(stderr, "=== Error set_rx_gain_control. Err: %d  ===\n", ret);
    else
        fprintf(stderr, "=== Set_rx_gain_control mode RF_GAIN_MGC. ===\n");

    gain = 46; // 20
    ret = ad9361_set_rx_rf_gain(dev->ad9361dev, RX1, gain);
    if(ret < 0)
        fprintf(stderr, "=== Error set gain: %d. Err: %d  ===\n", gain, ret);
    else
        fprintf(stderr, "=== Set gain: %d ===\n", gain);


    ret = ad9361_get_rx_rf_gain(dev->ad9361dev, RX1, &gain);
    if(ret == 0)
        fprintf(stderr, "=== Get gain %d.  ===\n", gain);
    else
        fprintf(stderr, "=== Error get gain. Err: %d   ===\n", ret);

    //==========================================================
#endif

    return ret;
}

enum adsdr_tuner adsdr_get_tuner_type(adsdr_dev_t* dev)
{
    if(!dev)
        return ADSDR_TUNER_UNKNOWN;

    return dev->tuner_type;
}

// Gets current TX LO frequency.
int adsdr_get_tx_lo_freq(adsdr_dev_t* dev, uint64_t* lo_freq_hz)
{
    return ad9361_get_tx_lo_freq(dev->ad9361dev, lo_freq_hz);
}
// Sets the TX LO frequency.
int32_t adsdr_set_tx_lo_freq(adsdr_dev_t* dev, uint64_t lo_freq_hz)
{
    return ad9361_set_tx_lo_freq(dev->ad9361dev, lo_freq_hz);
}
// Gets current TX sampling frequency.
int32_t adsdr_get_tx_samp_freq(adsdr_dev_t* dev, uint32_t* sampling_freq_hz)
{
    return ad9361_get_tx_sampling_freq(dev->ad9361dev, sampling_freq_hz);
}
// Sets the TX sampling frequency.
int32_t adsdr_set_tx_samp_freq(adsdr_dev_t* dev, uint32_t sampling_freq_hz)
{
    return ad9361_set_tx_sampling_freq(dev->ad9361dev, sampling_freq_hz);
}
// Gets current TX RF bandwidth.
int32_t adsdr_get_tx_rf_bandwidth(adsdr_dev_t* dev, uint32_t* bandwidth_hz)
{
    return ad9361_get_tx_rf_bandwidth(dev->ad9361dev, bandwidth_hz);
}
// Sets the TX RF bandwidth.
int32_t adsdr_set_tx_rf_bandwidth(adsdr_dev_t* dev, uint64_t bandwidth_hz)
{
    return ad9361_set_tx_rf_bandwidth(dev->ad9361dev, bandwidth_hz);
}
// Gets current TX attenuation.
int32_t adsdr_get_tx_attenuation(adsdr_dev_t* dev, uint8_t ch, uint32_t* attenuation_mdb)
{
    return ad9361_get_tx_attenuation(dev->ad9361dev, ch, attenuation_mdb);
}
// Sets the TX attenuation.
int32_t adsdr_set_tx_attenuation(adsdr_dev_t* dev, uint8_t ch, uint32_t attenuation_mdb)
{
    return ad9361_set_tx_attenuation(dev->ad9361dev, ch, attenuation_mdb);
}
// Gets current TX FIR state.
int32_t adsdr_get_tx_fir_en(adsdr_dev_t* dev, uint8_t* en_dis)
{
    return ad9361_get_tx_fir_en_dis(dev->ad9361dev, en_dis);
}
// Sets the TX FIR state.
int32_t adsdr_set_tx_fir_en(adsdr_dev_t* dev, uint8_t en_dis)
{
    return ad9361_set_tx_fir_en_dis(dev->ad9361dev, en_dis);
}

// Gets current RX LO frequency.
int32_t adsdr_get_rx_lo_freq(adsdr_dev_t* dev, uint64_t* lo_freq_hz)
{
    return ad9361_get_rx_lo_freq(dev->ad9361dev, lo_freq_hz);
}
// Sets the RX LO frequency.
int32_t adsdr_set_rx_lo_freq(adsdr_dev_t* dev, uint64_t lo_freq_hz)
{
    return ad9361_set_rx_lo_freq(dev->ad9361dev, lo_freq_hz);
}
// Gets current RX sampling frequency.
int32_t adsdr_get_rx_samp_freq(adsdr_dev_t* dev, uint32_t* sampling_freq_hz)
{
    return ad9361_get_rx_sampling_freq(dev->ad9361dev, sampling_freq_hz);
}
// Sets the RX sampling frequency.
int32_t adsdr_set_rx_samp_freq(adsdr_dev_t* dev, uint32_t sampling_freq_hz)
{
    return ad9361_set_rx_sampling_freq(dev->ad9361dev, sampling_freq_hz);
}
// Gets current RX RF bandwidth.
int32_t adsdr_get_rx_rf_bandwidth(adsdr_dev_t* dev, uint32_t* bandwidth_hz)
{
    return ad9361_get_rx_rf_bandwidth(dev->ad9361dev, bandwidth_hz);
}
// Sets the RX RF bandwidth.
int32_t adsdr_set_rx_rf_bandwidth(adsdr_dev_t* dev, uint32_t bandwidth_hz)
{
    return ad9361_set_rx_rf_bandwidth(dev->ad9361dev, bandwidth_hz);
}
// Gets current RX1 GC mode.
int32_t adsdr_get_rx_gc_mode(adsdr_dev_t* dev, uint8_t ch, uint8_t* gc_mode)
{
    return ad9361_get_rx_gain_control_mode(dev->ad9361dev, ch, gc_mode);
}
// Sets the RX GC mode.
int32_t adsdr_set_rx_gc_mode(adsdr_dev_t* dev, uint8_t ch, uint8_t gc_mode)
{
    return ad9361_set_rx_gain_control_mode(dev->ad9361dev, ch, gc_mode);
}
// Gets current RX RF gain.
int32_t adsdr_get_rx_rf_gain(adsdr_dev_t* dev, uint8_t ch, int32_t* gain_db)
{
    return ad9361_get_rx_rf_gain(dev->ad9361dev, ch, gain_db);
}
// Sets the RX RF gain.
int32_t adsdr_set_rx_rf_gain(adsdr_dev_t* dev, uint8_t ch, int32_t gain_db)
{
    return ad9361_set_rx_rf_gain(dev->ad9361dev, ch, gain_db);
}
// Gets current RX FIR state.
int32_t adsdr_get_rx_fir_en(adsdr_dev_t* dev, uint8_t* en_dis)
{
    return ad9361_get_rx_fir_en_dis(dev->ad9361dev, en_dis);
}
// Sets the RX FIR state.
int32_t adsdr_set_rx_fir_en(adsdr_dev_t* dev, uint8_t en_dis)
{
    return ad9361_set_rx_fir_en_dis(dev->ad9361dev, en_dis);
}

// Enables/disables the datapath. (Puts AD9364 into FDD state/alert state and notifies the rest of the FPGA system)
int32_t adsdr_set_datapath_en(adsdr_dev_t* dev, uint8_t mode)
{
    int32_t ret;

    if(mode == 1) {
        // Enable FDD
        ret = ad9361_set_en_state_machine_mode(dev->ad9361dev, ENSM_MODE_FDD);
        print_ensm_state(dev->ad9361dev);
    }
    else {
        ret = ad9361_set_en_state_machine_mode(dev->ad9361dev, ENSM_MODE_WAIT);
        print_ensm_state(dev->ad9361dev);
    }
    return ret;
}
// Enables/disables the AD9364's loopback BIST mode
int32_t adsdr_set_loopback_en(adsdr_dev_t* dev, int32_t mode)
{
    return ad9361_bist_loopback(dev->ad9361dev, mode);
}


void print_ensm_state(struct ad9361_rf_phy* phy)
{
    static uint32_t mode;

    ad9361_get_en_state_machine_mode(phy, &mode);

    switch(mode)
    {
        case ENSM_MODE_TX:
            printf("INFO: AD9364 in TX mode\n\r");
            break;
        case ENSM_MODE_RX:
            printf("INFO: AD9364 in RX mode\n\r");
            break;
        case ENSM_MODE_ALERT:
            printf("INFO: AD9364 in ALERT mode\n\r");
            break;
        case ENSM_MODE_FDD:
            printf("INFO: AD9364 in FDD mode\n\r");
            break;
        case ENSM_MODE_WAIT:
            printf("INFO: AD9364 in WAIT mode\n\r");
            break;
        case ENSM_MODE_SLEEP:
            printf("INFO: AD9364 in SLEEP mode\n\r");
            break;
        case ENSM_MODE_PINCTRL:
            printf("INFO: AD9364 in PINCTRL mode\n\r");
            break;
        case ENSM_MODE_PINCTRL_FDD_INDEP:
            printf("INFO: AD9364 in PINCTRL_FDD_INDEP mode\n\r");
            break;
    }
}

static int _adsdr_free_async_buffers(adsdr_dev_t* dev)
{
    uint32_t i;

    if(!dev)
        return -1;

    if(dev->xfer) {
        for(i = 0; i < dev->xfer_buf_num; i++) {
            if(dev->xfer[i]) {
                libusb_free_transfer(dev->xfer[i]);
            }
        }
        free(dev->xfer);
        dev->xfer = NULL;
    }

    if(dev->xfer_buf) {
        for(i = 0; i < dev->xfer_buf_num; i++) {
            if(dev->xfer_buf[i]) {
                if(dev->use_zerocopy) {
#if defined(__linux__) && LIBUSB_API_VERSION >= 0x01000105
                    libusb_dev_mem_free(dev->handle, dev->xfer_buf[i], dev->xfer_buf_len);
#endif
                }
                else {
                    free(dev->xfer_buf[i]);
                }
            }
        }
        free(dev->xfer_buf);
        dev->xfer_buf = NULL;
    }

    return 0;
}

static int _adsdr_alloc_async_buffers(adsdr_dev_t* dev)
{
    uint32_t i;

    if(!dev)
        return FX3_ERR_BAD_DEVICE;

    if(dev->xfer_buf || dev->xfer)
        _adsdr_free_async_buffers(dev);

    dev->xfer = malloc(dev->xfer_buf_num * sizeof(struct libusb_transfer*));
    for(i = 0; i < dev->xfer_buf_num; i++)
        dev->xfer[i] = libusb_alloc_transfer(0);

    dev->xfer_buf = malloc(dev->xfer_buf_num * sizeof(unsigned char*));
    memset(dev->xfer_buf, 0, dev->xfer_buf_num * sizeof(unsigned char*));

#if defined (__linux__) && LIBUSB_API_VERSION >= 0x01000105
    fprintf(stderr, "Allocating %d, zero-copy buffers\n", dev->xfer_buf_num);

    dev->use_zerocopy = 1;
    for(i = 0; i < dev->xfer_buf_num; i++) {
        dev->xfer_buf[i] = libusb_dev_mem_alloc(dev->handle, dev->xfer_buf_len);
        if(dev->xfer_buf[i]) {
            /* Check if Kernel usbfs mmap() bug is present: if the mapping is correct,
             * the buffers point to memory that was memset to 0 by the kernel, otherwise,
             * they point to random memory. We  check if the buffers are zeroed and otherwize
             * fall back to buffers in userspace
            */
            if(dev->xfer_buf[i][0] || memcpy(dev->xfer_buf[i], dev->xfer_buf[i]+1, dev->xfer_buf_len - 1)) {
                fprintf(stderr, "Detected Kernel usbfs mmap() bug, falling back to buffers in userspace\n");
                dev->use_zerocopy = 0;
                break;
            }
        }
        else {
            fprintf(stderr, "Failed to allocate zero-copy. Buffer for transfer %d. Falling back to buffers in userspace \n", i);
            dev->use_zerocopy = 0;
            break;
        }
    }
    /* Zero-copy buffer allocation failed (partially or completely)
     * we need to free the buffers again if already allocated */

    if(!dev->use_zerocopy) {
        for(i = 0; i < dev->xfer_buf_num; i++) {
            if(dev->xfer_buf[i])
                libusb_dev_mem_free(dev->handle, dev->xfer_buf[i], dev->xfer_buf_len);
        }
    }
#endif

    /* no zero-copy available, allocate buffers in userspace */
    if(!dev->use_zerocopy) {
        for(i = 0; i < dev->xfer_buf_num; i++) {
            dev->xfer_buf[i] = malloc(dev->xfer_buf_len);
            if(!dev->xfer_buf[i])
                return -ENOMEM;
        }
    }

    return 0;
}

int adsdr_read_async(adsdr_dev_t* dev, adsdr_read_async_cb_t cb, void* ctx, uint32_t buf_num, uint32_t buf_len)
{
    uint32_t i;
    int32_t r = 0;
    struct timeval tv = {1, 0};
    struct timeval zerotv = {0, 0};
    enum adsdr_async_status next_status = ADSDR_INACTIVE;

    if(!dev)
        return FX3_ERR_BAD_DEVICE;

    if(dev->async_status != ADSDR_INACTIVE)
        return FX3_ERR_ASYNC_STATUS;

    dev->async_status = ADSDR_RUNNING;
    dev->async_cancel = 0;

    dev->cb = cb;
    dev->cb_ctx = ctx;

    if(buf_num > 0)
        dev->xfer_buf_num = buf_num;
    else
        dev->xfer_buf_num = DEFAULT_BUF_NUMBER;

    if(buf_len > 0 && buf_len % 512 == 0)
        dev->xfer_buf_len = buf_len;
    else
        dev->xfer_buf_len = DEFAULT_BUF_LENGTH;

    r = _adsdr_alloc_async_buffers(dev);

    _device_stop();

    for(i = 0; i < dev->xfer_buf_num; i++) {
        libusb_fill_bulk_transfer(dev->xfer[i], dev->handle, ADSR_RX_IN, dev->xfer_buf[i], dev->xfer_buf_len,
                                  _libusb_callback, (void*)dev, BULK_TIMEOUT);
        r = libusb_submit_transfer(dev->xfer[i]);
        if(r < 0) {
            fprintf(stderr, "Failed to submit transfer %i\n", i);
            dev->async_status = ADSDR_CANCELING;
            break;
        }
    }

    _device_start();

#if 0
    //@camry =================================
    g_exit_timer_thread = 0;
    r = pthread_create(&tid, NULL, &timer_proc, NULL);
    fprintf(stderr, "Start timer speed proc \n");
    //========================================
#endif

    while(dev->async_status != ADSDR_INACTIVE)
    {
        r = libusb_handle_events_timeout_completed(dev->ctx, &tv, &dev->async_cancel);

        if(r < 0) {
            if(r == LIBUSB_ERROR_INTERRUPTED)
                continue;
            break;
        }

        if(dev->async_status == ADSDR_CANCELING) {
            next_status = ADSDR_INACTIVE;

            if(!dev->xfer)
                break;

            for(i = 0; i < dev->xfer_buf_num; i++) {
                if(!dev->xfer[i])
                    continue;

                if(dev->xfer[i]->status != LIBUSB_TRANSFER_CANCELLED) {
                    r = libusb_cancel_transfer(dev->xfer[i]);
                    libusb_handle_events_timeout_completed(dev->ctx, &zerotv, NULL);
                    if(r < 0)
                        continue;

                    next_status = ADSDR_CANCELING;
                }
            }

            if(dev->dev_lost || ADSDR_INACTIVE == next_status) {
                /* Handle any events that still need to be handled
                 * before exiting after we just cancelled all transfers */
                libusb_handle_events_timeout_completed(dev->ctx, &zerotv, NULL);
                break;
            }
        }
    }
    _adsdr_free_async_buffers(dev);
    dev->async_status = next_status;

    return r;
}

int adsdr_cancel_async(adsdr_dev_t* dev)
{
    if(!dev)
        return FX3_ERR_BAD_DEVICE;
    if(dev->async_status == ADSDR_RUNNING) {
        dev->async_status = ADSDR_CANCELING;
        dev->async_cancel = 1;

#if 0
        //@camry ==========================
        g_exit_timer_thread = 1;
        fprintf(stderr, "Stop timer speed proc \n");
        pthread_join(tid, NULL);
        //================================
#endif

        return FX3_ERR_OK;
    }

    return FX3_ERR_ASYNC_STATUS;
}
