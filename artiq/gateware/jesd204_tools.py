from collections import namedtuple
from fractions import gcd

from migen import *
from migen.genlib.cdc import MultiReg
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

        sysref_freq = sample_rate / (2 * ts.k * ts.s) # check factor of 2!
        sysref_div = int(refclk_freq/sysref_freq)

        self.ibuf_disable = CSRStorage(reset=1)
        self.jreset = CSRStorage(reset=1)
        self.sysref_out_en = CSRStorage(reset=0)

        self.sysref_ctr = Signal(4)
        # sysref_ctr = Signal(max=sysref_div-1)
        self.sysref = Signal()
        sysref_out = Signal(reset_less=True)
        sysref_out_en = Signal(reset_less=True)

        self.refclk = Signal()
        self.clock_domains.cd_jesd = ClockDomain()
        refclk2 = Signal()
        refclk_pads = platform.request("dac_refclk", 0)

        platform.add_period_constraint(refclk_pads.p, 1e9/self.refclk_freq)
        self.specials += [
            Instance("IBUFDS_GTE3", i_CEB=self.ibuf_disable.storage,
                     p_REFCLK_HROW_CK_SEL=0b00,
                     i_I=refclk_pads.p, i_IB=refclk_pads.n,
                     o_O=self.refclk, o_ODIV2=refclk2),
            AsyncResetSynchronizer(self.cd_jesd, self.jreset.storage),
        ]

        if use_rtio_clock:
            self.comb += self.cd_jesd.clk.eq(ClockSignal("rtio"))
        else:
            self.specials += Instance("BUFG_GT", i_I=refclk2,
                                      o_O=self.cd_jesd.clk)

        # self.specials += MultiReg(self.sysref_out_en.storage, sysref_out_en)
        # self.sync.rtio += [
        #     sysref_ctr.eq(sysref_ctr-1),
        #     If(self.sysref, sysref_ctr.eq(sysref_div-1))
        # ]
        self.comb += self.sysref.eq(self.sysref_ctr == 0)
        self.sync.rtio += sysref_out.eq(self.sysref)

        pads = platform.request("sma_io", 0)
        self.comb += pads.level.eq(sysref_out)
        self.comb += pads.direction.eq(1)


PhyPads = namedtuple("PhyPads", "txp txn")


class UltrascaleTX(Module, AutoCSR):
    def __init__(self, platform, sys_crg, jesd_crg, dac, rtio_clk):

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
                rtio_clk,
                phy.transmitter.cd_tx.clk)
            phys.append(phy)

        self.submodules.core = JESD204BCoreTX(
            phys, jesd_crg.jesd_settings, converter_data_width=64)
        self.submodules.control = JESD204BCoreTXControl(self.core)
        self.core.register_jsync(platform.request("dac_sync", dac))
        self.core.register_jref(jesd_crg.sysref)
