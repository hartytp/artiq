from migen import *
from migen.genlib.cdc import PulseSynchronizer, MultiReg
from migen.genlib.fsm import FSM
from misoc.interconnect.csr import *


class DDMTDSamplerExtFF(Module):
    def __init__(self, ddmtd_inputs):
        self.rec_clk = Signal()
        self.main_xo = Signal()

        # # #

        # TODO: s/h timing at FPGA pads
        if hasattr(ddmtd_inputs, "rec_clk"):
            rec_clk_1 = ddmtd_inputs.rec_clk
        else:
            rec_clk_1 = Signal()
            self.specials += Instance("IBUFDS",
                i_I=ddmtd_inputs.rec_clk_p, i_IB=ddmtd_inputs.rec_clk_n,
                o_O=rec_clk_1)
        if hasattr(ddmtd_inputs, "main_xo"):
            main_xo_1 = ddmtd_inputs.main_xo
        else:
            main_xo_1 = Signal()
            self.specials += Instance("IBUFDS",
                i_I=ddmtd_inputs.main_xo_p, i_IB=ddmtd_inputs.main_xo_n,
                o_O=main_xo_1)
        self.specials += [
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=rec_clk_1, o_Q=self.rec_clk,
                attr={("IOB", "TRUE")}),
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=main_xo_1, o_Q=self.main_xo,
                attr={("IOB", "TRUE")}),
        ]


class DDMTDSamplerGTP(Module):
    def __init__(self, gtp, main_xo_pads):
        self.rec_clk = Signal()
        self.main_xo = Signal()

        # # #

        # Getting the main XO signal from IBUFDS_GTE2 is problematic because
        # the transceiver PLL craps out if an improper clock signal is applied,
        # so we are disabling the buffer until the clock is stable.
        main_xo_se = Signal()
        rec_clk_1 = Signal()
        main_xo_1 = Signal()
        self.specials += [
            Instance("IBUFDS",
                i_I=main_xo_pads.p, i_IB=main_xo_pads.n,
                o_O=main_xo_se),
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=gtp.cd_rtio_rx0.clk, o_Q=rec_clk_1,
                attr={("DONT_TOUCH", "TRUE")}),
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=rec_clk_1, o_Q=self.rec_clk,
                attr={("DONT_TOUCH", "TRUE")}),
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=main_xo_se, o_Q=main_xo_1,
                attr={("IOB", "TRUE")}),
            Instance("FD", i_C=ClockSignal("helper"),
                i_D=main_xo_1, o_Q=self.main_xo,
                attr={("DONT_TOUCH", "TRUE")}),
        ]


class DDMTDDeglitcherFirstEdge(Module):
    def __init__(self, input_signal, blind_period=5000):
        self.detect = Signal()
        self.tag_correction = 0

        rising = Signal()
        input_signal_r = Signal()
        self.sync.helper += [
            input_signal_r.eq(input_signal),
            rising.eq(input_signal & ~input_signal_r)
        ]

        blind_counter = Signal(max=blind_period)
        self.sync.helper += [
            If(blind_counter != 0, blind_counter.eq(blind_counter - 1)),
            If(input_signal_r, blind_counter.eq(blind_period - 1)),
            self.detect.eq(rising & (blind_counter == 0))
        ]


class DDMTD(Module):
    def __init__(self, counter, input_signal):

        # in helper clock domain
        self.h_tag = Signal(len(counter))
        self.h_tag_update = Signal()

        # # #

        deglitcher = DDMTDDeglitcherFirstEdge(input_signal)
        self.submodules += deglitcher

        self.sync.helper += [
            self.h_tag_update.eq(0),
            If(deglitcher.detect,
                self.h_tag_update.eq(1),
                self.h_tag.eq(counter + deglitcher.tag_correction)
            )
        ]


class Collector(Module):
    """Generates loop filter inputs from DDMTD outputs.

    The input to the main DCXO lock loop filter is the difference between the
    reference and main tags after unwrapping (see below).

    The input to the helper DCXO lock loop filter is the difference between the
    current reference tag and the previous reference tag after unwrapping.

    When the WR PLL is locked, the following ideally (no noise/jitter) obtain:
    - f_main = f_ref
    - f_helper = f_ref * 2^N/(2^N+1)
    - f_beat = f_ref - f_helper = f_ref / (2^N + 1) (cycle time is: dt=1/f_beat)
    - the reference and main DCXO tags are equal to each other at every cycle
      (the main DCXO lock drives this difference to 0)
    - the reference and main DCXO tags both have the same value at each cycle
      (the tag difference for each DDMTD is given by
      f_helper*dt = f_helper/f_beat = 2^N, which causes the N-bit DDMTD counter
      to wrap around and come back to its previous value)

    Note that we currently lock the frequency of the helper DCXO to the
    reference clock, not it's phase. As a result, while the tag differences are
    controlled, their absolute values are arbitrary. We could consider moving
    the helper lock to a phase lock at some point in the future...

    Since the DDMTD counter is only N bits, it is possible for tag values to
    wrap around. This will happen frequently if the locked tags happens to be
    near the edges of the counter, so that jitter can easily cause a phase wrap.
    But, it can also easily happen during lock acquisition or other transients.
    To avoid glitches in the output, we unwrap the tag differences. Currently
    we do this in hardware, but we should consider extending the processor to
    allow us to do it inside the filters. Since the processor uses wider
    signals, this would significantly extend the overall glitch-free
    range of the PLL and may aid lock acquisition.
    """
    def __init__(self):
        self.ref_stb = Signal()
        self.main_stb = Signal()
        self.tag_ref = Signal(64)
        self.tag_main = Signal(64)

        self.out_stb = Signal()
        self.out_main = Signal((48, True))
        self.out_helper = Signal((48, True))
        self.out_tag_ref = Signal(64)
        self.out_tag_main = Signal(64)

        tag_ref_r = Signal((64, True))
        tag_main_r = Signal((64, True))
        tag_ref_expected = Signal((64, True), reset=2**15)
        tag_main_expected = Signal((64, True), reset=2**15)
        tag_ref_diff = Signal((64, True))
        tag_main_diff = Signal((64, True))
        tag_ref_offset = Signal((64, True))
        tag_main_offset = Signal((64, True))

        # While the PLL is locking we can easily get a second of one tag while
        # we are waiting for the other (either due to jitter or due to one
        # frequency being higher than the other)
        ref_early = Signal()
        main_early = Signal()
        ref_stb = Signal()
        main_stb = Signal()

        # # #

        self.comb += [
            ref_stb.eq(ref_early | self.ref_stb),
            main_stb.eq(main_early | self.main_stb)
        ]

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        fsm.act("IDLE",
            NextValue(self.out_stb, 0),
            NextValue(ref_early, 0),
            NextValue(main_early, 0),

            If(ref_stb & main_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextValue(tag_main_r, self.tag_main),
                NextState("DIFF")
            ).Elif(ref_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextState("WAITMAIN")
            ).Elif(main_stb,
                NextValue(tag_main_r, self.tag_main),
                NextState("WAITREF")
            )
        )
        fsm.act("WAITREF",
            If(self.main_stb, NextValue(main_early, 1)),
            If(self.ref_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextState("DIFF")
            )
        )
        fsm.act("WAITMAIN",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb,
                NextValue(tag_main_r, self.tag_main),
                NextState("DIFF")
            )
        )
        fsm.act("DIFF",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb, NextValue(main_early, 1)),

            NextValue(tag_ref_expected, tag_ref_r + 2**15),
            NextValue(tag_main_expected, tag_main_r + 2**15),
            NextValue(tag_ref_diff, tag_ref_r - tag_ref_expected),
            NextValue(tag_main_diff, tag_main_r - tag_main_expected),

            NextState("UNWRAP")
        )
        fsm.act("UNWRAP",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb, NextValue(main_early, 1)),

            If(tag_ref_diff > 2**14,
               NextValue(tag_ref_offset, tag_ref_offset - 2**15),
               NextValue(self.out_helper, tag_ref_diff - 2**15),
            ).Else(NextValue(self.out_helper, tag_ref_diff)),
            If(tag_main_diff > 2**14,
               NextValue(tag_main_offset, tag_main_offset - 2**15),
            ),
            NextState("UNWRAP2")
        )
        fsm.act("UNWRAP2",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb, NextValue(main_early, 1)),

            If(tag_ref_diff < -2**14,
               NextValue(tag_ref_offset, tag_ref_offset + 2**15),
               NextValue(self.out_helper, tag_ref_diff + 2**15),
            ),
            If(tag_main_diff < -2**14,
               NextValue(tag_main_offset, tag_main_offset + 2**15),
            ),

            NextState("UNWRAP3")
        )
        fsm.act("UNWRAP3",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb, NextValue(main_early, 1)),

            NextValue(tag_ref_r, tag_ref_r + tag_ref_offset),
            NextValue(tag_main_r, tag_main_r + tag_main_offset),

            NextState("OUTPUT")
        )
        fsm.act("OUTPUT",
            If(self.ref_stb, NextValue(ref_early, 1)),
            If(self.main_stb, NextValue(main_early, 1)),

            NextValue(self.out_tag_ref, tag_ref_r),
            NextValue(self.out_tag_main, tag_main_r),
            NextValue(self.out_main, tag_main_r - tag_ref_r),
            NextValue(self.out_stb, 1),
            NextState("IDLE")
        )
