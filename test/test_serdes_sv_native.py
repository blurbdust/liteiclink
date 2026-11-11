#
# This file is part of LiteICLink.
#
# Copyright (c) 2026 blurbdust <blurbdust@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

import random
import unittest

from migen import *

from litex.soc.cores.prbs import PRBS7Generator, PRBS15Generator, PRBS31Generator

from liteiclink.serdes.sv_native import SVNativePRBSTX, SVNativePRBSRX, PRBS7, PRBS15, PRBS31
from liteiclink.serdes.sv_native import _PRBSGenerator, _PRBS_TAPS

class _Link(Module):
    """PRBS generator into checker through a model of the serial line: TX words are sent bit 0
    first and the receiver cuts the bit stream into words at an arbitrary offset, as a PMA without
    word alignment does."""
    def __init__(self, width, offset):
        self.submodules.tx = SVNativePRBSTX(width)
        self.submodules.rx = SVNativePRBSRX(width, lock_words=16)
        self.width  = width
        self.offset = offset

def _run(width, offset, tx_cfg, rx_cfg, tx_rev=1, rx_rev=1, tx_inv=0, rx_inv=0, inject_at=(), cycles=400):
    dut = _Link(width, offset)
    results = {}

    def gen():
        yield dut.tx.config.eq(tx_cfg)
        yield dut.rx.config.eq(rx_cfg)
        yield dut.tx.reverse.eq(tx_rev)
        yield dut.rx.reverse.eq(rx_rev)
        yield dut.tx.invert.eq(tx_inv)
        yield dut.rx.invert.eq(rx_inv)
        bits = []
        for cycle in range(cycles):
            yield dut.tx.inject.eq(cycle in inject_at)
            if cycle == 100:
                yield dut.rx.clear.eq(1)
                results["locked_before_clear"] = (yield dut.rx.locked)
            else:
                yield dut.rx.clear.eq(0)
            word = (yield dut.tx.o)
            bits += [(word >> b) & 1 for b in range(width)]
            if len(bits) >= offset + width:
                rx = sum(bit << b for b, bit in enumerate(bits[offset:offset + width]))
                bits = bits[width:]
                yield dut.rx.i.eq(rx)
            yield
        # Read before the line stops: a stuck input is an error too.
        results["words"]       = (yield dut.rx.words)
        results["error_words"] = (yield dut.rx.error_words)
        results["error_bits"]  = (yield dut.rx.error_bits)
        results["locked"]      = (yield dut.rx.locked)

    run_simulation(dut, gen())
    return results

class TestSVNativePRBS(unittest.TestCase):
    def test_generator_matches_litex(self):
        for prbs, ref in [(PRBS7, PRBS7Generator), (PRBS15, PRBS15Generator), (PRBS31, PRBS31Generator)]:
            for width in [32, 40, 64]:
                n, taps = _PRBS_TAPS[prbs]
                dut = Module()
                dut.submodules.a = a = ref(width)
                dut.submodules.b = b = _PRBSGenerator(width, n, taps)
                words = []
                def gen():
                    for _ in range(20):
                        yield
                        words.append(((yield a.o), (yield b.o)))
                run_simulation(dut, gen())
                self.assertTrue(all(x == y for x, y in words), (prbs, width))

    def test_prbs_any_offset(self):
        cases  = [(40, prbs, offset) for prbs in [PRBS7, PRBS15, PRBS31] for offset in [0, 1, 17, 39]]
        cases += [(80, PRBS31, offset) for offset in [0, 37]]
        for width, prbs, offset in cases:
            r = _run(width, offset, prbs, prbs)
            self.assertEqual(r["error_words"], 0, (width, prbs, offset, r))
            self.assertEqual(r["locked"], 1)
            self.assertGreater(r["words"], 250)

    def test_bit_order_matters(self):
        # The serialiser sends bit 0 first: without reversing, words cut at an offset do not check.
        r = _run(40, 13, PRBS31, PRBS31, tx_rev=0, rx_rev=0)
        self.assertGreater(r["error_words"], 100)
        self.assertEqual(r["locked"], 0)

    def test_pattern_mismatch(self):
        r = _run(40, 5, PRBS7, PRBS31)
        self.assertGreater(r["error_words"], 100)
        self.assertEqual(r["locked"], 0)

    def test_polarity(self):
        self.assertGreater(_run(40, 3, PRBS31, PRBS31, tx_inv=1)["error_words"], 100)
        self.assertEqual(_run(40, 3, PRBS31, PRBS31, tx_inv=1, rx_inv=1)["error_words"], 0)

    def test_inject(self):
        # One flipped bit is wrong itself and corrupts the two later predictions that tap it.
        r = _run(80, 21, PRBS31, PRBS31, inject_at=(200,))
        self.assertEqual(r["locked"], 1)
        self.assertEqual(r["error_bits"], 3)
        self.assertIn(r["error_words"], [1, 2])

    def test_clear(self):
        r = _run(40, 0, PRBS31, PRBS31)
        self.assertEqual(r["locked_before_clear"], 1)
        self.assertLess(r["words"], 310)

if __name__ == "__main__":
    unittest.main()
