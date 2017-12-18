"""
Drivers for direct digital synthesis (DDS) AD9910(Urukul) on RTIO.

Output event replacement is not supported and issuing commands at the same
time is an error.

Only the basic set of the functions inplemented and tested: 
CPLD configuration
CPLD status check
AD9910 initiallization and profile 0 (single tone)
Attenuator control
SW control (EMM1 must be connected)

Note: current code version can't work with the accurate update timing and
the DDS synchroniztaion. The SPI gateware need to be fixed 1st.
"""


from artiq.language.core import *
from artiq.language.types import *
from artiq.language.units import *
from artiq.coredevice.exceptions import DDSError

from numpy import int32, int64
import numpy

from artiq.coredevice import spi


_SPI_CONFIG = (0*spi.SPI_OFFLINE | 1*spi.SPI_CS_POLARITY |
                      0*spi.SPI_CLK_POLARITY | 0*spi.SPI_CLK_PHASE |
                      0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)

#not implemented
_PHASE_MODE_DEFAULT   = -1
PHASE_MODE_CONTINUOUS = 0
PHASE_MODE_ABSOLUTE   = 1
PHASE_MODE_TRACKING   = 2

_SPIT_CFG_WR = 5
_SPIT_STATUS_RD = 16
_SPIT_DDS_WR = 128
_SPIT_DDS_RD = 128
_SPIT_ATT_WR = 5

_CFG_LE = (1 << 11)
_CFG_CLK_SEL = (1 << 17)
_CFG_RST = (1 << 19)
_CFG_IO_RST = (1 << 20)

_CFG_RF_SW_OFS = 0
_CFG_PROFILE_OFS = 8

_STA_PROTO_REV_OFS = 16
_STA_PROTO_REV_MASK = 0xFF
_STA_PROTO_REV_MATCH = 0x06

_CS_CFG = 1
_CS_ATT = 2
_CS_DDS_OFS = 4

_AD9910_REG_CFR1 = 0x00
_AD9910_REG_CFR2 = 0x01
_AD9910_REG_CFR3 = 0x02
_AD9910_REG_AUX_DAC = 0x03
_AD9910_REG_IO_UPD = 0x04
_AD9910_REG_FTW = 0x07
_AD9910_REG_POW = 0x08
_AD9910_REG_ASF = 0x09
_AD9910_REG_MSYNC = 0x0A
_AD9910_REG_DRAMPL = 0x0B
_AD9910_REG_DRAMPS = 0x0C
_AD9910_REG_DRAMPR = 0x0D
_AD9910_REG_PR0 = 0x0E
_AD9910_REG_PR1 = 0x0F
_AD9910_REG_PR2 = 0x10
_AD9910_REG_PR3 = 0x11
_AD9910_REG_PR4 = 0x12
_AD9910_REG_PR5 = 0x13
_AD9910_REG_PR6 = 0x14
_AD9910_REG_PR7 = 0x15
_AD9910_REG_RAM = 0x16

class DDSParams:
    def __init__(self):
        self.channel     = 0
        self.ftw         = 0
        self.pow         = 0
        self.amplitude   = 0

class BatchContextManager:
    kernel_invariants = {"core", "dds_urukul", "params"}

    def __init__(self, dds_urukul):
        self.dds_urukul = dds_urukul
        self.core     = self.dds_urukul.core
        self.active   = False
        self.params   = [DDSParams() for _ in range(16)]
        self.count    = 0
        self.ref_time = int64(0)

    @kernel
    def __enter__(self):
        """Starts a DDS command batch. All DDS commands are buffered
        after this call, until ``batch_exit`` is called.

        The time of execution of the DDS commands is the time cursor position
        when the batch is entered."""
        if self.active:
            raise DDSError("DDS batch entered twice")

        self.active   = True
        self.count    = 0
        self.ref_time = now_mu()

    @kernel
    def append(self, channel, ftw, pow, amplitude):
        if self.count == len(self.params):
            raise DDSError("Too many commands in DDS batch")

        params = self.params[self.count]
        params.channel     = channel
        params.ftw         = ftw
        params.pow         = pow
        params.amplitude   = amplitude
        self.count += 1

    @kernel
    def __exit__(self, type, value, traceback):
        """Ends a DDS command batch. All buffered DDS commands are issued
        on the bus."""
        if not self.active:
            raise DDSError("DDS batch exited twice")

        self.active = False
        at_mu(self.ref_time - self.dds_urukul.batch_duration_mu())
        for i in range(self.count):
            param = self.params[i]
            self.dds_urukul.program(self.ref_time,
                                  param.channel, param.ftw,
                                  param.pow, param.amplitude)


class DDSGroupUrukul:
    """Driver for AD9910 DDS chips. See ``DDSGroup`` for a description
    of the functionality."""
    kernel_invariants = {
        "pow_width", "rtio_period_mu", "sysclk_per_mu", "write_duration_mu",
        "dac_cal_duration_mu", "init_duration_mu", "init_sync_duration_mu",
        "program_duration_mu",
        "spi_device", "io_update_device", "dds_reset_device",
        "dds_channel_count",
    }

    pow_width = 16

    def __init__(self, dmgr, sysclk,
            ref_clk, clk_sel,
            dds_channel_count,
            spi_device, io_update_device, dds_reset_device,
            sync_clk_device=None, sync_in_device=None,
            io_update_ret_device=None, nu_mosi3_device=None,
            sw0_device=None, sw1_device=None, sw2_device=None,
            sw3_device=None,
            core_device="core",
        ):

        self.core   = dmgr.get(core_device)
        self.sysclk = sysclk
        self.batch  = BatchContextManager(self)

        self.bus = dmgr.get(spi_device)
        self.io_update = dmgr.get(io_update_device)
        self.dds_reset = dmgr.get(dds_reset_device)
        if sync_clk_device is not None:
            self.sync_clk = dmgr.get(sync_clk_device)
        if sync_in_device is not None:
            self.sync_in = dmgr.get(sync_in_device)
        if io_update_ret_device is not None:
            self.io_update_ret = dmgr.get(io_update_ret_device)
        if sw0_device is not None:
            self.sw0 = dmgr.get(sw0_device)
        if sw1_device is not None:
            self.sw1 = dmgr.get(sw1_device)
        if sw2_device is not None:
            self.sw2 = dmgr.get(sw2_device)
        if sw3_device is not None:
            self.sw3 = dmgr.get(sw3_device)

        self.ref_clk = ref_clk
        self.clk_sel = clk_sel

        self.dds_channel_count     = dds_channel_count

        self.spi_device = spi_device
        self.io_update_device      = io_update_device
        self.dds_reset_device      = dds_reset_device

        self.rtio_period_mu        = int64(8)
        self.sysclk_per_mu         = int32(self.sysclk * self.core.ref_period)

        self.write_duration_mu     = 5 * self.rtio_period_mu  #optimize SPI clk and adjust
        self.dac_cal_duration_mu   = 147000 * self.rtio_period_mu  #optimize SPI clk and adjust
        self.init_duration_mu      = 1000*self.rtio_period_mu #optimize SPI clk and adjust
        self.init_sync_duration_mu = 16 * self.write_duration_mu + 2 * self.dac_cal_duration_mu #optimize SPI clk and adjust
        self.program_duration_mu   = 6 * self.write_duration_mu #optimize SPI clk and adjust

        self.cpld_config_reg = numpy.int32(0)
        self.att_reg = numpy.int32(0)
        self.status_reg = numpy.int32(0)
        self.proto_rev = 0xFF

    @kernel
    def write_cfg(self):
        self.bus.set_config_mu(_SPI_CONFIG,
                               _SPIT_CFG_WR, _SPIT_STATUS_RD)
        self.bus.set_xfer(_CS_CFG, 24, 0)
        self.bus.write(self.cpld_config_reg << 8)

    @kernel
    def read_status(self):
        self.bus.set_config_mu(_SPI_CONFIG,
                               _SPIT_STATUS_RD, _SPIT_STATUS_RD)
        self.bus.set_xfer(_CS_CFG, 24, 0)
        self.bus.write(self.cpld_config_reg << 8)
        self.bus.read_async()
        delay(20*us) #not optimized
        return self.bus.input_async()

    @kernel
    def init(self):
        delay(10*ms)
        self.cpld_config_reg = (_CFG_RST
                | (_CFG_CLK_SEL if self.clk_sel == 1 else 0))
        self.write_cfg()
        self.cpld_config_reg &= ~_CFG_RST;
        self.write_cfg()
        self.status_reg = self.read_status()
        self.proto_rev = ((self.status_reg >> _STA_PROTO_REV_OFS)
                & _STA_PROTO_REV_MASK)
        if self.proto_rev != _STA_PROTO_REV_MATCH:
            raise DDSError("Unexpected proto_rev")

    @kernel
    def rf_sw(self, sw, bit):
        self.cpld_config_reg &= ~(1 << (URUKUL_CFG_RF_SW_OFS + sw))
        self.cpld_config_reg |= (
               1 << (URUKUL_CFG_RF_SW_OFS + sw) if bit == 1 else 0)
        self.write_cfg()

    @kernel
    def io_rst(self):
        self.cpld_config_reg |= _CFG_IO_RST
        self.write_cfg()
        delay(1*us) #optimize SPI clk and adjust
        self.cpld_config_reg &= ~_CFG_IO_RST;
        self.write_cfg()
        delay(1*us) #optimize SPI clk and adjust

    @kernel
    def write_dds_reg(self, dds, reg, data, data2=0):
        """Write 8 + 32 bits of data.
    
        This method advances the timeline by the duration of the SPI transfer
        and the required CS high time.
        """
        if dds >= 4 or reg > 31:
            raise ValueError("DDS register out of range")

        self.bus.set_config_mu(_SPI_CONFIG,
                _SPIT_DDS_WR, _SPIT_DDS_WR)
        self.bus.set_xfer(4 + dds, 8, 0)
        self.bus.write(reg << 24)
        self.bus.set_xfer(4 + dds, 32, 0)
        if reg >= 0x0E:
            self.bus.write(data)
            self.bus.write(data2)
        else:
            self.bus.write(data)
        self.io_update.pulse(100*ns)

    @kernel
    def read_dds_reg(self, dds, reg):
        if dds >= 4 or reg > 31:
            raise ValueError("DDS register out of range")

        self.io_rst()
        self.bus.set_config_mu(_SPI_CONFIG,
                _SPIT_DDS_WR, _SPIT_DDS_WR)
        self.bus.set_xfer(4 + dds, 8, 0)
        self.bus.write((0x80 | reg) << 24)
        delay_mu(200*self.bus.ref_period_mu)  #optimize SPI clk and adjust
        self.bus.set_config_mu(_SPI_CONFIG,
                _SPIT_DDS_RD, _SPIT_DDS_RD)
        self.bus.set_xfer(4 + dds, 0, 32)
        self.bus.write(0)
        delay_mu(self.bus.ref_period_mu)  #optimize SPI clk and adjust
        self.bus.read_async()
        return self.bus.input_async()

    @kernel
    def dds_init(self, dds, pll_n, pll_cp):
        if dds >= 4:
             raise ValueError("DDS register out of range")
        self.write_dds_reg(dds, _AD9910_REG_CFR1, 0x00000002)
        reg3 = self.read_dds_reg(dds, 3)
        delay(1*ms) #optimize SPI clk and adjust
        if reg3 != 0x7f7f:
            raise DDSError("AD9910 not detected")
        self.write_dds_reg(dds, _AD9910_REG_CFR2, 0x00400820 | (1 << 24))
        pll_freq = self.ref_clk * pll_n
        if pll_freq < 420000000:
            raise ValueError("pll_n is too low")
        if pll_freq > 1000000000 or pll_n >= 127:
            raise ValueError("pll_n is too high")
        if pll_freq > 920000000:
            pll_vco_sel = 5
        elif pll_freq > 832000000:
            pll_vco_sel = 4
        elif pll_freq > 656000000:
            pll_vco_sel = 3
        elif pll_freq > 562000000:
            pll_vco_sel = 2
        elif pll_freq > 482000000:
            pll_vco_sel = 1
        else: 
            pll_vco_sel = 0
        if pll_cp > 7:
            raise ValueError("pll_cp is too high")
        self.write_dds_reg(dds, _AD9910_REG_CFR3, (0x01 << 28) |
                (1 << 27) | (pll_vco_sel << 24) | (pll_cp << 19) |
                (7 << 16) | (1 << 14) | (1 << 8) | (pll_n << 1))
        delay(10*ms) #PLL lock, need to set more accurate or do the polling
        self.write_cfg() #to latch the actual value
        if (self.read_status() & (1 << (8 + dds))) == 0:
            raise DDSError("PLL lock fail")
        return pll_freq

    @kernel
    def set_att(self, dds, att):
        if dds >= 4 or att > 63:
             raise ValueError("ATT settings out of range")

        self.att_reg &= ~(0x3F << ((dds << 3) + 2))
        self.att_reg |= att << ((dds << 3) + 2)
        self.bus.set_config_mu(_SPI_CONFIG,
                _SPIT_ATT_WR, _SPIT_ATT_WR)
        self.bus.set_xfer(_CS_ATT, 32, 0)
        self.bus.write(self.att_reg)
        delay(1*us) #optimize SPI clk and adjust
        self.cpld_config_reg |= _CFG_LE
        self.write_cfg()
        delay(100*ns) #optimize SPI clk and adjust
        self.cpld_config_reg &= ~_CFG_LE;
        self.write_cfg()

    @kernel
    def test_att_noise(self):
        for i in range(0, 1024):
            delay(5*us) #optimize SPI clk and adjust
            self.bus.set_config_mu(_SPI_CONFIG,
                    _SPIT_ATT_WR, _SPIT_ATT_WR)
            self.bus.set_xfer(_CS_ATT, 32, 0)
            self.bus.write(self.att_reg)

    @kernel
    def test_dds_noise(self):
        for i in range(0, 1024):
            delay(2*us) #optimize SPI clk and adjust
            self.write_dds_reg(3, _AD9910_REG_PR7, 0, 0) 

    @kernel
    def set(self, channel, ftw, pow, phase_mode, amplitude):
        ref_time = 0
        self.program(ref_time,
                     channel, ftw, pow, amplitude)

    @portable(flags={"fast-math"})
    def frequency_to_ftw(self, frequency):
        """Returns the frequency tuning word corresponding to the given
        frequency.
        """
        return round(float(int64(2)**32*frequency/self.sysclk))

    @portable(flags={"fast-math"})
    def ftw_to_frequency(self, ftw):
        """Returns the frequency corresponding to the given frequency tuning
        word.
        """
        return ftw*self.sysclk/int64(2)**32

    @portable(flags={"fast-math"})
    def turns_to_pow(self, turns):
        """Returns the phase offset word corresponding to the given phase
        in turns."""
        return round(float(turns*2**self.pow_width))

    @portable(flags={"fast-math"})
    def pow_to_turns(self, pow):
        """Returns the phase in turns corresponding to the given phase offset
        word."""
        return pow/2**self.pow_width

    @portable(flags={"fast-math"})
    def amplitude_to_asf(self, amplitude):
        """Returns amplitude scale factor corresponding to given amplitude."""
        return round(float(amplitude*0x3ffe))

    @portable(flags={"fast-math"})
    def asf_to_amplitude(self, asf):
        """Returns the amplitude corresponding to the given amplitude scale
           factor."""
        return asf/0x3ffe

    @kernel
    def batch_duration_mu(self):
        return self.batch.count * (self.program_duration_mu +
                                   self.write_duration_mu) # + FUD time

    @kernel
    def program(self, ref_time, channel, ftw, pow, amplitude):
        if amplitude >= 0x3FFF:
             raise ValueError("Amplitude is out of range")
        if pow > 0xFFFF:
             raise ValueError("pow is out of range")
        self.write_dds_reg(channel,
                _AD9910_REG_PR0, (amplitude << 16) | pow, ftw
            ) #need to add the profiles 1-7 setup


class DDSChannelAD9910:
    kernel_invariants = {
        "core", "dds_urukul", "channel", "pll_n", "pll_cp"
    }

    def __init__(self, dmgr, channel, pll_n, pll_cp, dds_urukul="dds_urukul"):
        self.dds_urukul    = dmgr.get(dds_urukul)
        self.core        = self.dds_urukul.core
        self.channel     = channel
        self.phase_mode  = 0
        self.pll_n = pll_n
        self.pll_cp = pll_cp
        self.pll_freq = 0
        self.switches = [
            self.dds_urukul.sw0, self.dds_urukul.sw1,
            self.dds_urukul.sw2, self.dds_urukul.sw3,
        ]

    @kernel
    def init(self):
        """Resets and initializes the DDS channel.

        This needs to be done for each DDS channel before it can be used, and
        it is recommended to use the startup kernel for this.

        This function cannot be used in a batch; the correct way of
        initializing multiple DDS channels is to call this function
        sequentially with a delay between the calls. 2ms provides a good
        timing margin."""
        self.pll_freq = self.dds_urukul.dds_init(
                self.channel, self.pll_n, self.pll_cp)

    @kernel
    def write_reg(self, reg, data, data2=0):
        self.dds_urukul.write_dds_reg(self.channel, reg, data, data2)

    @kernel
    def read_reg(self, reg):
        self.dds_urukul.read_dds_reg(self.channel, reg)

    @kernel
    def set_att(self, att):
        self.dds_urukul.set_att(self.channel, att)

    @kernel
    def on(self):
        self.switches[self.channel].on()

    @kernel
    def off(self):
        self.switches[self.channel].off()

    @kernel
    def pulse(self, duration):
        self.switches[self.channel].pulse(duration)

    @kernel
    def pulse_mu(self, duration):
        self.switches[self.channel].pulse_mu(duration)

    @kernel
    def set_mu(self, frequency, phase=0, amplitude=0x3fff):
        """Sets the DDS channel to the specified frequency and phase.

        This uses machine units (FTW and POW). The frequency tuning word width
        is 32, whereas the phase offset word width depends on the type of DDS
        chip and can be retrieved via the ``pow_width`` attribute. The amplitude
        width is 14.

        The "frequency update" pulse is sent to the DDS with a fixed latency
        with respect to the current position of the time cursor.

        :param frequency: frequency to generate.
        :param phase: adds an offset, in turns, to the phase.
        :param phase_mode: if specified, overrides the default phase mode set
            by ``set_phase_mode`` for this call.
        """
        self.dds_urukul.set(self.channel, frequency, phase, self.phase_mode, amplitude)

    @kernel
    def set(self, frequency, phase=0.0, amplitude=1.0):
        """Like ``set_mu``, but uses Hz and turns."""
        self.set_mu(self.dds_urukul.frequency_to_ftw(frequency),
                    self.dds_urukul.turns_to_pow(phase),
                    self.dds_urukul.amplitude_to_asf(amplitude))
