
#include <climits>
#include <cstring>

#include "RtlSdrSource.h"

using namespace std;


// Open RTL-SDR device.
RtlSdrSource::RtlSdrSource(int dev_index)
    : m_dev(0)
    , m_block_length(default_block_length)
{
	 m_dev = NULL;
	 rx0_i = NULL;
	 rx0_q = NULL;
	 rxbuf = NULL;
	 prx_buffer = NULL;

	m_dev = iio_create_network_context("192.168.2.1");
    if (m_dev == NULL) {
        m_error =  "Failed to open nh7020 device (";
        m_error += errno;
        m_error += ")";
    }
}


// Close RTL-SDR device.
RtlSdrSource::~RtlSdrSource()
{
	if (m_dev) {
		if (rx0_i) { iio_channel_disable(rx0_i); }
		if (rx0_q) { iio_channel_disable(rx0_q); }
		iio_context_destroy(m_dev);
	}
}


// Configure RTL-SDR tuner and prepare for streaming.
bool RtlSdrSource::configure(uint32_t sample_rate,
                             uint32_t frequency,
                             int tuner_gain,
                             int block_length,
                             bool agcmode)
{
    int r;

    if (!m_dev)
        return false;


	iio_device *phydev = iio_context_find_device(m_dev, "ad9361-phy");
	struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", false);
	iio_channel_attr_write(phy_chn, "rf_port_select", "A_BALANCED");
	iio_channel_attr_write_longlong(phy_chn, "rf_bandwidth", sample_rate);
	iio_channel_attr_write_longlong(phy_chn, "sampling_frequency", sample_rate);

	

	struct iio_channel* phy = iio_device_find_channel(phydev, "altvoltage0", true);
    r = iio_channel_attr_write_longlong(phy, "frequency", (uint64_t)frequency);

    if (r < 0) {
        m_error = "set frequency failed";
        return false;
    }

    if (tuner_gain == INT_MIN) {
		iio_channel_attr_write(phy_chn, "gain_control_mode", "slow_attack");
        if (r < 0) {
            m_error = "hardwaregain could not set automatic gain";
            return false;
        }
    } else {
        r = iio_channel_attr_write(phy_chn, "gain_control_mode", "manual");
        if (r < 0) {
            m_error = "gain_control_mode could not set manual gain";
            return false;
        }

		r = iio_channel_attr_write_double(phy_chn, "hardwaregain", tuner_gain);
        if (r < 0) {
            m_error = "set hardwaregain failed";
            return false;
        }
    }

    // set RTL AGC mode
    // set block length
    m_block_length = (block_length < 4096) ? 4096 :
                     (block_length > 1024 * 1024) ? 1024 * 1024 :
                     block_length;
    m_block_length -= m_block_length % 4096;

    // reset buffer to start streaming

	iio_device *pluto_rx = iio_context_find_device(m_dev, "cf-ad9361-lpc");

	rx0_i = iio_device_find_channel(pluto_rx, "voltage0", false);
	rx0_q = iio_device_find_channel(pluto_rx, "voltage1", false);
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);

	rxbuf = iio_device_create_buffer(pluto_rx, m_block_length, false);
	prx_buffer = (short *)iio_buffer_start(rxbuf);

    return true;
}


// Return current sample frequency in Hz.
uint32_t RtlSdrSource::get_sample_rate()
{
	long long int sample_rate = 0;
	iio_device *phydev = iio_context_find_device(m_dev, "ad9361-phy");
	struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", false);
	iio_channel_attr_read_longlong(phy_chn, "sampling_frequency",&sample_rate);
    return sample_rate;
}


// Return current center frequency in Hz.
uint32_t RtlSdrSource::get_frequency()
{
	long long int frequency = 0;
	iio_device *phydev = iio_context_find_device(m_dev, "ad9361-phy");
	struct iio_channel* phy_chn = iio_device_find_channel(phydev, "altvoltage0", true);
	iio_channel_attr_read_longlong(phy_chn, "frequency", &frequency);
	return frequency;
}


// Return current tuner gain in units of 1 dB.
int RtlSdrSource::get_tuner_gain()
{
	double gain = 0;
	iio_device *phydev = iio_context_find_device(m_dev, "ad9361-phy");
	struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", false);
	iio_channel_attr_read_double(phy_chn, "hardwaregain", &gain);
    return gain;
}


// Return a list of supported tuner gain settings in units of 1 dB.
vector<int> RtlSdrSource::get_tuner_gains()
{
	int num_gains = 80;
	if (num_gains <= 0)
		return vector<int>();

	vector<int> gains(num_gains);

	for (int i = 0; i < num_gains; i++) {
		gains[i] = i;
	}

	return gains;
}


// Fetch a bunch of samples from the device.
bool RtlSdrSource::get_samples(IQSampleVector& samples)
{
    int r, n_read;

    if (!m_dev)
        return false;
	
    r = n_read = iio_buffer_refill(rxbuf);
    if (r < 0) {
        m_error = "iio_buffer_refill failed";
        return false;
    }
/*
    if (n_read != 2 * m_block_length) {
        m_error = "short read, samples lost";
        return false;
    }
*/
    samples.resize(m_block_length);
    for (int i = 0; i < m_block_length; i++) {
		int idx = i * 2;
        int16_t re = prx_buffer[idx];
		int16_t im = prx_buffer[idx +1];

        samples[i] = IQSample( re / IQSample::value_type(2047),
                               im / IQSample::value_type(2047) );
    }

    return true;
}


// Return a list of supported devices.
vector<string> RtlSdrSource::get_device_names()
{
    vector<string> result;

    int device_count = 1;
    if (device_count <= 0)
        return result;

    result.reserve(device_count);
    for (int i = 0; i < device_count; i++) {
        result.push_back(string("NH7020"));
    }

    return result;
}

/* end */
