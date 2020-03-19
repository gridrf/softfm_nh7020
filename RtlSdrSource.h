#ifndef SOFTFM_RTLSDRSOURCE_H
#define SOFTFM_RTLSDRSOURCE_H

#include <cstdint>
#include <string>
#include <vector>

#include "SoftFM.h"
#include "iio.h"


class RtlSdrSource
{
public:

    static const int default_block_length = 65536;

    /** Open RTL-SDR device. */
    RtlSdrSource(int dev_index);

    /** Close RTL-SDR device. */
    ~RtlSdrSource();

    /**
     * Configure RTL-SDR tuner and prepare for streaming.
     *
     * sample_rate  :: desired sample rate in Hz.
     * frequency    :: desired center frequency in Hz.
     * tuner_gain   :: desired tuner gain in 0.1 dB, or INT_MIN for auto-gain.
     * block_length :: preferred number of samples per block.
     *
     * Return true for success, false if an error occurred.
     */
    bool configure(std::uint32_t sample_rate,
                   std::uint32_t frequency,
                   int tuner_gain,
                   int block_length=default_block_length,
                   bool agcmode=false);

    /** Return current sample frequency in Hz. */
    std::uint32_t get_sample_rate();

    /** Return current center frequency in Hz. */
    std::uint32_t get_frequency();

    /** Return current tuner gain in units of 0.1 dB. */
    int get_tuner_gain();

    /** Return a list of supported tuner gain settings in units of 0.1 dB. */
    std::vector<int> get_tuner_gains();

    /** Return name of opened RTL-SDR device. */
    std::string get_device_name() const
    {
        return m_devname;
    }

    /**
     * Fetch a bunch of samples from the device.
     *
     * This function must be called regularly to maintain streaming.
     * Return true for success, false if an error occurred.
     */
    bool get_samples(IQSampleVector& samples);

    /** Return the last error, or return an empty string if there is no error. */
    std::string error()
    {
        std::string ret(m_error);
        m_error.clear();
        return ret;
    }

    /** Return true if the device is OK, return false if there is an error. */
    operator bool() const
    {
        return m_dev && m_error.empty();
    }

    /** Return a list of supported devices. */
    static std::vector<std::string> get_device_names();

private:
    struct iio_context * m_dev;
	struct iio_channel *rx0_i;
	struct iio_channel *rx0_q;
	struct iio_buffer  *rxbuf;
	short *prx_buffer;
    int                 m_block_length;
    std::string         m_devname;
    std::string         m_error;
};

#endif
