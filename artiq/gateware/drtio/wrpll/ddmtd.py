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
    def __init__(self, input_signal, blind_period=128):
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
            If(rising, blind_counter.eq(blind_period - 1)),
            self.detect.eq(rising & (blind_counter == 0))
        ]


class DDMTD(Module, AutoCSR):
    def __init__(self, counter, input_signal):
        self.arm = CSR()
        self.tag = CSRStatus(len(counter))

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

        tag_update_ps = PulseSynchronizer("helper", "sys")
        self.submodules += tag_update_ps
        self.comb += tag_update_ps.i.eq(self.h_tag_update)
        tag_update = Signal()
        self.sync += tag_update.eq(tag_update_ps.o)

        tag = Signal(len(counter))
        self.h_tag.attr.add("no_retiming")
        self.specials += MultiReg(self.h_tag, tag)

        self.sync += [
            If(self.arm.re & self.arm.r, self.arm.w.eq(1)),
            If(tag_update,
                If(self.arm.w, self.tag.status.eq(tag)),
                self.arm.w.eq(0),
            )
        ]


class Collector(Module):
    """ Generates loop filter inputs from DDMTD outputs.

    When the WR PLL is locked, the following ideally (no noise etc) obtain:
    - f_main = f_ref
    - f_helper = f_ref * (2^N-1) / 2^N
    - f_beat = f_ref - f_helper = f_ref / 2^N (cycle time is: dt=1/f_beat)
    - the reference and main DCXO tags are equal each cycle
    - the reference and main DCXO tags decrease by 1 each cycle (the tag
      difference is f_helper*dt = f_helper/f_beat = (2^N-1) so we are 1 tag
      away from a complete wrap around of the N-bit DDMTD counter)

    Since the main and reference tags cycle through all possible values when
    locked, we need to unwrap the collector outputs to avoid glitches
    (particularly noise around transitions). Currently we do this in hardware,
    but we should consider extending the processor to allow us to do it
    inside the filters at a later stage (at which point, the collector
    essentially becomes a the trigger for the loop filters).

    The input to the main DCXO lock loop filter is the difference between the
    reference and main tags after phase unwrapping.

    The input to the helper DCXO lock loop filter is the difference between the
    current reference tag and the previous reference tag plus 1, after
    phase unwrapping.
    """
    def __init__(self, N):
        self.ref_stb = Signal()
        self.main_stb = Signal()
        self.tag_ref = Signal(N)
        self.tag_main = Signal(N)

        self.out_stb = Signal()
        self.out_main = Signal((N+2, True))
        self.out_helper = Signal((N+2, True))
        self.out_tag_ref = Signal(N)
        self.out_tag_main = Signal(N)

        tag_ref_r = Signal(N)
        tag_main_r = Signal(N)
        main_tag_diff = Signal((N+2, True))
        helper_tag_diff = Signal((N+2, True))

        # # #

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        fsm.act("IDLE",
            NextValue(self.out_stb, 0),
            If(self.ref_stb & self.main_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextValue(tag_main_r, self.tag_main),
                NextState("DIFF")
            ).Elif(self.ref_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextState("WAITMAIN")
            ).Elif(self.main_stb,
                NextValue(tag_main_r, self.tag_main),
                NextState("WAITREF")
            )
        )
        fsm.act("WAITREF",
            If(self.ref_stb,
                NextValue(tag_ref_r, self.tag_ref),
                NextState("DIFF")
            )
        )
        fsm.act("WAITMAIN",
            If(self.main_stb,
                NextValue(tag_main_r, self.tag_main),
                NextState("DIFF")
            )
        )
        fsm.act("DIFF",
            NextValue(main_tag_diff, tag_main_r - tag_ref_r),
            NextValue(helper_tag_diff, tag_ref_r - self.out_tag_ref + 1),
            NextState("UNWRAP")
        )
        fsm.act("UNWRAP",
            If(main_tag_diff - self.out_main > 2**(N-1),
               NextValue(main_tag_diff, main_tag_diff - 2**N)
            ).Elif(self.out_main - main_tag_diff > 2**(N-1),
               NextValue(main_tag_diff, main_tag_diff + 2**N)
            ),

            If(helper_tag_diff - self.out_helper > 2**(N-1),
               NextValue(helper_tag_diff, helper_tag_diff - 2**N)
            ).Elif(self.out_helper - helper_tag_diff > 2**(N-1),
               NextValue(helper_tag_diff, helper_tag_diff + 2**N)
            ),

            NextState("OUTPUT")
        )
        fsm.act("OUTPUT",
            NextValue(self.out_tag_ref, tag_ref_r),
            NextValue(self.out_tag_main, tag_main_r),
            NextValue(self.out_main, main_tag_diff),
            NextValue(self.out_helper, helper_tag_diff),
            NextValue(self.out_stb, 1),
            NextState("IDLE")
        )


def test_collector_main():

    N = 2
    collector = Collector(N=N)
    # check collector phase unwrapping
    tags = [(0, 0, 0),
            (0, 1, 1),
            (2, 1, -1),
            (3, 1, -2),
            (0, 1, -3),
            (1, 1, -4),
            (2, 1, -5),
            (3, 1, -6),
            (3, 3, -4),
            (0, 0, -4),
            (0, 1, -3),
            (0, 2, -2),
            (0, 3, -1),
            (0, 0, 0)]
    for i in range(10):
        tags.append((i % (2**N), (i+1) % (2**N), 1))

    def generator():
        for tag_ref, tag_main, out in tags:
            yield collector.tag_ref.eq(tag_ref)
            yield collector.tag_main.eq(tag_main)
            yield collector.main_stb.eq(1)
            yield collector.ref_stb.eq(1)

            yield

            yield collector.main_stb.eq(0)
            yield collector.ref_stb.eq(0)

            while not (yield collector.out_stb):
                yield

            out_main = yield collector.out_main
            assert out_main == out

    run_simulation(collector, generator())


def test_collector_helper():

    N = 3
    collector = Collector(N=N)
    # check collector phase unwrapping
    tags = [((2**N - 1 - tag) % (2**N), 0) for tag in range(20)]
    tags += [((tags[-1][0] + 1 + tag) % (2**N), 2) for tag in range(20)]
    tags += [((tags[-1][0] - 2 - 2*tag) % (2**N), -1) for tag in range(20)]

    def generator():
        for tag_ref, out in tags:
            yield collector.tag_ref.eq(tag_ref)
            yield collector.main_stb.eq(1)
            yield collector.ref_stb.eq(1)

            yield

            yield collector.main_stb.eq(0)
            yield collector.ref_stb.eq(0)

            while not (yield collector.out_stb):
                yield

            out_helper = yield collector.out_helper
            assert out_helper == out

    run_simulation(collector, generator())

if __name__ == "__main__":
    test_collector_main()
    test_collector_helper()
