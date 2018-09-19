from collections import namedtuple
import numpy as np

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from misoc.interconnect.csr import *

from jesd204b.common import (JESD204BTransportSettings,
                             JESD204BPhysicalSettings,
                             JESD204BSettings)
from jesd204b.phy.gth import GTHChannelPLL as JESD204BGTHChannelPLL
from jesd204b.phy import JESD204BPhyTX
from jesd204b.core import JESD204BCoreTX
from jesd204b.core import JESD204BCoreTXControl


class UltrascaleCRG(Module, AutoCSR):

    def __init__(self, platform, sample_rate, fabric_freq, refclk_freq,
                 use_rtio_clock=False):

        ps = self.jesd_ps = JESD204BPhysicalSettings(l=8, m=4, n=16, np=16)
        ts = self.jesd_ts = JESD204BTransportSettings(f=2, s=2, k=16, cs=0)
        self.jesd_settings = JESD204BSettings(ps, ts, did=0x5a, bid=0x5)

        self.linerate = int(20*sample_rate*ps.m/ps.l)
        self.refclk_freq = int(refclk_freq)
        self.fabric_freq = int(fabric_freq)

        sysref_freq = sample_rate * ts.s / (ts.k * ts.f)
        sysref_div = int(np.log2(fabric_freq/sysref_freq))

        assert sysref_freq == fabric_freq / (1 << sysref_div)
        assert sysref_div >= 1

        if refclk_freq == fabric_freq:
            clk_sel = 0b00
        elif refclk_freq == 2*fabric_freq:
            clk_sel = 0b01
        else:
            raise ValueError("Invalid refclk frequency")

        self.jref_en = CSRStorage(reset=0)
        self.ibuf_disable = CSRStorage(reset=1)
        self.jreset = CSRStorage(reset=1)

        self.jref_ctr = Signal(sysref_div)
        self.jref = Signal()
        self.jref_out = Signal(reset_less=True)
        self.refclk = Signal()

        self.clock_domains.cd_jesd = ClockDomain()

        refclk2 = Signal()
        refclk_pads = platform.request("dac_refclk", 0)
        platform.add_period_constraint(refclk_pads.p, 1e9/self.refclk_freq)
        self.specials += [
            Instance("IBUFDS_GTE3", i_CEB=self.ibuf_disable.storage,
                     p_REFCLK_HROW_CK_SEL=clk_sel,
                     i_I=refclk_pads.p, i_IB=refclk_pads.n,
                     o_O=self.refclk, o_ODIV2=refclk2),
            AsyncResetSynchronizer(self.cd_jesd, self.jreset.storage),
        ]

        if use_rtio_clock:
            self.comb += self.cd_jesd.clk.eq(ClockSignal("rtio"))
        else:
            self.specials += Instance("BUFG_GT",
                                      i_I=refclk2,
                                      o_O=self.cd_jesd.clk)

        self.sync.rtio += self.jref_ctr.eq(self.jref_ctr + 1)
        self.comb += self.jref.eq(self.jref_ctr[-1])
        self.sync.jesd += self.jref_out.eq(self.jref & self.jref_en.storage)

        sysref_out = platform.request("sma_io", 0)
        self.comb += sysref_out.level.eq(self.jref_out)
        self.comb += sysref_out.direction.eq(1)


PhyPads = namedtuple("PhyPads", "txp txn")


class UltrascaleTX(Module, AutoCSR):
    def __init__(self, platform, sys_crg, jesd_crg, dac):

        jesd_pads = platform.request("dac_jesd", dac)
        phys = []
        for i in range(len(jesd_pads.txp)):
            cpll = JESD204BGTHChannelPLL(
                    jesd_crg.refclk, jesd_crg.refclk_freq, jesd_crg.linerate)
            self.submodules += cpll
            phy = JESD204BPhyTX(
                    cpll, PhyPads(jesd_pads.txp[i], jesd_pads.txn[i]),
                    jesd_crg.fabric_freq, transceiver="gth")
            platform.add_period_constraint(phy.transmitter.cd_tx.clk,
                                           40*1e9/jesd_crg.linerate)
            platform.add_false_path_constraints(
                sys_crg.cd_sys.clk,
                jesd_crg.cd_jesd.clk,
                phy.transmitter.cd_tx.clk)
            phys.append(phy)

        self.submodules.core = JESD204BCoreTX(
            phys, jesd_crg.jesd_settings, converter_data_width=128)
        self.submodules.control = JESD204BCoreTXControl(self.core)
        self.core.register_jsync(platform.request("dac_sync", dac))
        self.core.register_jref(jesd_crg.jref)
