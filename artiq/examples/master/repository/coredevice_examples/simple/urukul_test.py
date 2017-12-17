#experiment for the Urukul AD9910 analog performance testing

from artiq.experiment import *
import numpy

class URUKULTest(EnvExperiment):

    def build(self):
        self.setattr_device("core")
        self.setattr_device("fmcdio_dirctl")
        self.setattr_device("dds_urukul")
        self.setattr_device("dds_ad9910_0")
        self.setattr_device("dds_ad9910_1")
        self.setattr_device("dds_ad9910_2")
        self.setattr_device("dds_ad9910_3")
        self.setattr_device("led")

    def p(self, f, *a):
        print(f % a)

    @kernel
    def run(self):
        self.core.reset()
        delay(5*ms)  # build slack for shift register set
        self.fmcdio_dirctl.set(0x0A008800) # added urukul MISO
            # (LVDS pair 2) and urukul_io_update_ret (LVDS pair 10)
        self.led.on()
        delay(5*ms)
        self.led.off()

        self.dds_urukul.init()
        self.dds_ad9910_0.init()
        self.dds_ad9910_1.init()
        self.dds_ad9910_2.init()
        self.dds_ad9910_3.init()
       
        delay(10*ms)

        self.dds_ad9910_0.set(10*MHz+0.2*Hz, 0.0, 0.5)
        self.dds_ad9910_0.set_att(20)
        self.dds_ad9910_0.on()

        self.dds_ad9910_1.set(10*MHz, 0.0, 0.5)
        self.dds_ad9910_1.set_att(20)
        self.dds_ad9910_1.on()

        self.dds_ad9910_2.set(10*MHz)
        self.dds_ad9910_2.set_att(20)
        self.dds_ad9910_2.on()

        self.dds_ad9910_3.set(10*MHz, 0.0, 0.3)
        self.dds_ad9910_3.set_att(20)
        self.dds_ad9910_3.on()

        self.led.pulse(10*ms)

        while False:                          # enable for the interference tests      
            delay(100*ns) #dummy
#            self.dds_urukul.test_dds_noise() # uncomment for the interference test for the ATT SPI
#            self.dds_urukul.test_att_noise() # uncomment for the interference test for the DDS3 SPI
