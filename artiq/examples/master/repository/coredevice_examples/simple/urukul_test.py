from artiq.experiment import *
from artiq.coredevice import spi

_URUKUL_SPI_CONFIG = (0*spi.SPI_OFFLINE | 1*spi.SPI_CS_POLARITY |   #SPI CS inverted for compatibility with URUKUL register select logic
                      0*spi.SPI_CLK_POLARITY | 1*spi.SPI_CLK_PHASE |
                      0*spi.SPI_LSB_FIRST | 0*spi.SPI_HALF_DUPLEX)

class URUKULTest(EnvExperiment):
    def build(self):
        self.setattr_device("core")
        self.setattr_device("fmcdio_dirctl")
        self.setattr_device("led")
        self.setattr_device("spi_urukul")
        self.setattr_device("urukul_io_update")
        self.setattr_device("urukul_dds_reset")
        self.setattr_device("urukul_sync_clk")
        self.setattr_device("urukul_sync_in")
        self.setattr_device("urukul_io_update_ret")
        self.setattr_device("urukul_nu_mosi3")
        self.setattr_device("urukul_sw0")
        self.setattr_device("urukul_sw1")
        self.setattr_device("urukul_sw2")
        self.setattr_device("urukul_sw3")

    @kernel
    def run(self):
        self.core.reset()
#        self.urukul_io_update_ret.output()
        delay(10*ms)  # build slack for shift register set
        self.fmcdio_dirctl.set(0x0A008800) # added urukul MISO (LVDS pair 2) and urukul_io_update_ret (LVDS pair 10)
        self.led.on()
        while True:
            self.led.pulse(50*ms)

            #basic SPI test
            self.spi_urukul.set_config_mu(_URUKUL_SPI_CONFIG, 30, 40)
            self.spi_urukul.set_xfer(7, 24, 0)
            self.spi_urukul.write(0xAAAAAAAA)

            #basic outputs test
            self.urukul_io_update.pulse(1*ms)
            delay(1*ms)
            self.urukul_dds_reset.pulse(2*ms)
            delay(1*ms)
            self.urukul_sync_clk.pulse(3*ms)
            delay(1*ms)
            self.urukul_sync_in.pulse(4*ms)
            delay(1*ms)
#            self.urukul_io_update_ret.pulse(5*ms)
#            delay(1*ms)
            self.urukul_nu_mosi3.pulse(6*ms)
            delay(1*ms)
            self.urukul_sw0.pulse(7*ms)
            delay(1*ms)
            self.urukul_sw1.pulse(8*ms)
            delay(1*ms)
            self.urukul_sw2.pulse(9*ms)
            delay(1*ms)
            self.urukul_sw3.pulse(10*ms)
            delay(1*ms)
